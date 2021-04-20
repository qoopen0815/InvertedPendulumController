#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);//シリアル通信開始、転送速度は9600ビット/秒
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Wire.begin();
  
  // Try to IMU initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double attitude = atan2(a.acceleration.x, a.acceleration.z);
  if (attitude > 0) attitude = - 3.14 - (3.14 - attitude);
  attitude -= -3.08;
  Serial.print(millis());
  Serial.print("\t");
  Serial.println(attitude);
}