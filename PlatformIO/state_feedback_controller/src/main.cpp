#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <I2Cdev.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <RotaryEncoder.h>
#include <TB6612FNG.h>
#include <Wire.h>

#define STBY 27
#define AIN1 12
#define AIN2 13
#define PWMA 25
#define BIN1 32
#define BIN2 33
#define PWMB 26

#define ENC1A 36
#define ENC1B 39
#define ENC2A 34
#define ENC2B 35

#define TESTPINA 5
#define TESTPINB 14
#define TESTPINC 18

// #define DEBUG_PIN_ON
// #define BLUETOOTH_ON

const float rate = 500.0;
const float dt = 1.0/rate;

BluetoothSerial SerialBT;
Kalman inclinometer;
MPU6050 imu;
RotaryEncoder encoder1(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder2(ENC2A, ENC2B, RotaryEncoder::LatchMode::TWO03);
Tb6612fng motors(STBY, AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);

// Inclinometer parameter
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t tempRaw;
float pitch, gyroY;
float kalAngleY; // Calculated angle using a Kalman filter
const float k = 0.7;

// Encoder parameter
float motorA;
float motorB;

float theta_p[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
float theta_w[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
float past_w = 0.0;
float lastCmd = 0.0;

const float K[4] = {25.13435117410369, 3.5851559194431846, 0.9018647125613708, 1.5374003579720874};  // Feedback Gain K

void useBluetooth( void *pvParameters );
void readInclinometer( void *pvParameters );
void readEncoder( void *pvParameters );
void updateMotorControl( void *pvParameters );

void setup()
{
  #ifdef DEBUG_PIN_ON
    pinMode(TESTPINA, OUTPUT);
    pinMode(TESTPINB, OUTPUT);
    pinMode(TESTPINC, OUTPUT);
    digitalWrite(TESTPINA, LOW);  /* LED1 off */
    digitalWrite(TESTPINB, LOW);  /* LED2 off */
    digitalWrite(TESTPINC, LOW);  /* LED2 off */
  #endif
  
  #ifdef BLUETOOTH_ON
    SerialBT.begin("ESP32");
  #endif

  Serial.begin(115200);
  Wire.begin();
  motors.begin();
  imu.initialize();

  // verify imu connection
  Serial.println("Testing device connections...");
  Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float pitch = atan2(ax, -az) * RAD_TO_DEG;
  inclinometer.setAngle(pitch);

  // Now set up two tasks to run independently.
  #ifdef BLUETOOTH_ON
    xTaskCreateUniversal( useBluetooth,
                          "useBluetooth",   // A name just for humans
                          1000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                          NULL,
                          2,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                          NULL,
                          PRO_CPU_NUM );
  #endif

  xTaskCreateUniversal( readInclinometer,
                        "readInclinometer",   // A name just for humans
                        1500,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        2,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        APP_CPU_NUM );
                        
  xTaskCreateUniversal( readEncoder,
                        "readEncoder",   // A name just for humans
                        1000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        1,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        APP_CPU_NUM );

  xTaskCreateUniversal( updateMotorControl,
                        "updateMotorControl",   // A name just for humans
                        1500,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        3,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        APP_CPU_NUM );
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void useBluetooth(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    SerialBT.print(micros());
    SerialBT.print("\t");
    SerialBT.println(pitch);
    vTaskDelay(3);  // one tick delay (15ms) in between reads for stability
  }
}

void readInclinometer(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    #ifdef DEBUG_PIN_ON
      digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
    #endif

    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    pitch = atan2(-ax, az) * RAD_TO_DEG;
    gyroY = radians( gy / 131.0 );  // Convert to deg/s
    kalAngleY = radians( inclinometer.getAngle(pitch, gyroY, 0.00042) + 2.0 );   // deg
    delayMicroseconds(100);
  }
}

void readEncoder(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    #ifdef DEBUG_PIN_ON
      digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
    #endif

    encoder1.tick();
    encoder2.tick();
    motorA = (float)encoder1.getPosition() * PI/600.;
    motorB = (float)encoder2.getPosition() * PI/600.;
    delayMicroseconds(50);
  }
}

void updateMotorControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    #ifdef DEBUG_PIN_ON
      digitalWrite(TESTPINC, !(digitalRead(TESTPINC)));
    #endif

    float control_input = K[0] * kalAngleY + 
                          K[1] * gyroY + 
                          K[2] * motorA + 
                          K[3] * (motorA - past_w)/dt;
    control_input = (1 - k) * lastCmd + k * control_input;
    lastCmd = control_input;
    past_w = motorA;
    control_input = min((double)control_input, 1.0);
    control_input = max((double)control_input, -1.0);
    motors.drive((float)control_input, (float)control_input);
    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}
