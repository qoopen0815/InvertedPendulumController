#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <I2Cdev.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <RotaryEncoder.h>
#include <TB6612FNG.h>
#include <Wire.h>

#define TESTPINA 5
#define TESTPINB 14

const double rate = 400.0;
const double dt = 1.0/rate;

Kalman inclinometer;
MPU6050 imu;
Tb6612fng motors(27, 12, 13, 25, 32, 33, 26);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t tempRaw;

double kalAngleY; // Calculated angle using a Kalman filter

// double theta[2] = {0.0, 0.0};  // 0: new, 1: old

void updateInclinometer( void *pvParameters );
void updateMotorControl( void *pvParameters );
void TaskView( void *pvParameters );

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
  xTaskCreateUniversal( updateInclinometer,
                        "updateInclinometer",   // A name just for humans
                        1000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        3,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        PRO_CPU_NUM );

  xTaskCreateUniversal( updateMotorControl,
                        "updateMotorControl",   // A name just for humans
                        800,           // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        2,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                        NULL,
                        PRO_CPU_NUM );

  xTaskCreateUniversal( TaskView,
                        "TaskView",     // A name just for humans
                        800,            // This stack size can be checked & adjusted by reading the Stack Highwater
                        NULL,
                        1,              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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

void updateInclinometer(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double pitch = atan2(-ax, az) * RAD_TO_DEG;
    double gyroY = gy / 131.0;  // Convert to deg/s
    kalAngleY = inclinometer.getAngle(pitch, gyroY, 30);
    digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}

void updateMotorControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    // Serial.println("Do");
    float speed = sin(0.0*3.14/180.0);
    motors.drive(speed, speed);
    digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
    vTaskDelay(8);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskView(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    // Serial.println("Do");
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}
