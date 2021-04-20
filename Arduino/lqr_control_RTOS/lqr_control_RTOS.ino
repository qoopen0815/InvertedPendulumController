#include <Arduino_FreeRTOS.h>
#include <RotaryEncoder.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

#include "Controller.h"
#include "TB6612FNG.h"

#define PIN_IN1 2
#define PIN_IN2 3

#define TESTPINA 5
#define TESTPINB 11

// MPU-6050のアドレス、レジスタ設定値
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

typedef struct {
  float ax; float ay; float az;
  float gx; float gy; float gz;
} Sensor;

const double rate = 200;
const double dt = 1.0/rate;

double a = 0.8;               //0<a<1の範囲　大きいほど効果大
int val = 0;                //センサ値
double rc = 0;

// define two tasks
void TaskGetIMU( void *pvParameters );
void TaskGetEncoder( void *pvParameters );
void TaskControl( void *pvParameters );

Sensor sensor;
Madgwick MadgwickFilter;
utils::TB6612FNG motor;
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
control::Bound bound = {7.0, -7.0};

double theta_p[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
double theta_w[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
double past_w, past_p;

double K[4] = { 27.37982666594578, 
                4.631113708545598, 
                0.8024493738303317, 
                1.4852798723539797};  // Feedback Gain K
// double K[4] = { 25.34658179454566, 
//                 4.301838094646338, 
//                 0.4857257565364246, 
//                 1.3793671339080538};  // Feedback Gain K

void setupIMU() {
  Wire.begin();

  // 初回の読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // 動作モードの読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  sensor.ax = 0.0; sensor.ay = 0.0; sensor.az = 0.0;
  sensor.gx = 0.0; sensor.gy = 0.0; sensor.gz = 0.0;
}

void updateIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  // 加速度値を分解能で割って加速度(G)に変換する
  sensor.ax = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  sensor.ay = ayRaw / 16384.0;
  sensor.az = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  sensor.gx = gxRaw / 131.0;//FS_SEL_0 131 LSB / (°/s)
  sensor.gy = gyRaw / 131.0;
  sensor.gz = gzRaw / 131.0;
}

void setup()
{
  Serial.begin(115200);
  MadgwickFilter.begin(66);
  setupIMU();
  motor.init();
  
  past_p = 0.0;
  past_w = 0.0;

  // Now set up two tasks to run independently.
  // xTaskCreate(
  //   TaskGetIMU
  //   ,  "TaskGetIMU"   // A name just for humans
  //   ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
  //   ,  NULL
  //   ,  1    // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //   ,  NULL );

  xTaskCreate(
    TaskGetEncoder
    ,  "TaskGetEncoder"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1    // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskControl
    ,  "TaskControl"
    ,  512  // Stack size
    ,  NULL
    ,  1    // Priority
    ,  NULL );
  
  pinMode(TESTPINA, OUTPUT);
  pinMode(TESTPINB, OUTPUT);

  delay(100);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskGetIMU(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    // digitalWrite(TESTPINA, !digitalRead(TESTPINA));
    updateIMU();
    MadgwickFilter.updateIMU( sensor.gx, sensor.gy, sensor.gz,
                              sensor.ax, sensor.ay, sensor.az );
    vTaskDelay(3);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskGetEncoder(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(TESTPINA, !digitalRead(TESTPINA));
    encoder.tick();
    delayMicroseconds(50);
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    digitalWrite(TESTPINB, !digitalRead(TESTPINB));
    
    updateIMU();
    MadgwickFilter.updateIMU( sensor.gx, sensor.gy, sensor.gz,
                              sensor.ax, sensor.ay, sensor.az );

    theta_p[0] = MadgwickFilter.getPitchRadians() - 0.034;
    theta_w[0] = (double)encoder.getPosition() * 3.14/600.;
    theta_p[1] = (theta_p[0] - past_p)/0.015;
    past_p += 0.015*theta_p[1];
    theta_w[1] = (theta_w[0] - past_w)/0.015;
    past_w += 0.015*theta_w[1];

    double control_input =  theta_p[0]*K[0] + theta_p[1]*K[1] + 
                            theta_w[2]*K[2] + theta_w[3]*K[3];        
    rc = a * rc + (1-a) * control_input;
    control_input = rc;

    control_input = min(control_input, bound.upper);
    control_input = max(control_input, bound.lower);
    motor.move((int)(control_input*255.0/7.0));

    // Serial.print(theta_p[0]);
    // Serial.print(",");
    // Serial.println(theta_p[1]);
    // Serial.print(",");
    // Serial.print(theta_w[0]);
    // Serial.print(",");
    // Serial.println(theta_w[1]);
    Serial.println(control_input*255.0/7.0);

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}