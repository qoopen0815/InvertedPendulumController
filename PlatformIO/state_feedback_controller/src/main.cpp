#include <Arduino.h>
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

const double rate = 400.0;
const double dt = 1.0/rate;

Kalman inclinometer;
MPU6050 imu;
RotaryEncoder encoder1(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder2(ENC2A, ENC2B, RotaryEncoder::LatchMode::TWO03);
Tb6612fng motors(STBY, AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);

// Inclinometer parameter
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t tempRaw;
double kalAngleY; // Calculated angle using a Kalman filter

// Encoder parameter
double motorA;
double motorB;

// double theta[2] = {0.0, 0.0};  // 0: new, 1: old

void readInclinometer( void *pvParameters );
void readEncoder( void *pvParameters );
void updateMotorControl( void *pvParameters );

void setup() {
  pinMode(TESTPINA, OUTPUT);
  pinMode(TESTPINB, OUTPUT);
  digitalWrite(TESTPINA, LOW);  /* LED1 off */
  digitalWrite(TESTPINB, LOW);  /* LED2 off */

  Serial.begin(115200);
  Wire.begin();
  motors.begin();
  imu.initialize();

  // verify imu connection
  Serial.println("Testing device connections...");
  Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double pitch = atan2(-ax, az) * RAD_TO_DEG;
  inclinometer.setAngle(pitch);

  // Now set up two tasks to run independently.
  xTaskCreateUniversal( readInclinometer,
                        "readInclinometer",   // A name just for humans
                        1000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        2,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        PRO_CPU_NUM );
                        
  xTaskCreateUniversal( readEncoder,
                        "readEncoder",   // A name just for humans
                        1000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        1,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        APP_CPU_NUM );

  xTaskCreateUniversal( updateMotorControl,
                        "updateMotorControl",   // A name just for humans
                        800,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        3,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        PRO_CPU_NUM );
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void readInclinometer(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double pitch = atan2(-ax, az) * RAD_TO_DEG;
    double gyroY = gy / 131.0;  // Convert to deg/s
    kalAngleY = inclinometer.getAngle(pitch, gyroY, 2);
    digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}

void readEncoder(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    encoder1.tick();
    encoder2.tick();
    motorA = (double)encoder1.getPosition() * 3.14/600.;
    motorB = (double)encoder2.getPosition() * 3.14/600.;
    digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
    delayMicroseconds(50);
  }
}

void updateMotorControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    float speed = sin(0.0*3.14/180.0);
    motors.drive(speed, speed);
    vTaskDelay(8);  // one tick delay (15ms) in between reads for stability
  }
}
