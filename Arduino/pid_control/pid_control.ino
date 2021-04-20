#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "TB6612FNG.h"
#include "Controller.h"

Adafruit_MPU6050 mpu;
utils::TB6612FNG motor;

const double rate = 100.0;
control::Bound bound = {255.0, -255.0};
control::Parameter param = {650.0, 15, 22};
control::PIDControl controller(param, bound, 1/rate);

#define PinA_left 2   // left encoder
#define PinA_right 4  // right encoder

long Pre_millis;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Wire.begin();
  motor.init();

  // Try to IMU initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Pre_millis = millis();
  delay(100);
}

void loop()
{
  sensors_event_t a, g, temp;
  if((millis()-Pre_millis) > (1000/rate)){ // Period: 10ms
    mpu.getEvent(&a, &g, &temp);
    const double attitude = atan2(a.acceleration.x, a.acceleration.z);
    const double input = controller.calculate(-0.06, attitude);
    Serial.print(attitude);
    Serial.println("");
    // Serial.print((int)input);
    // Serial.println("");
    motor.move((int)input);
    Pre_millis = millis();
  }
}
