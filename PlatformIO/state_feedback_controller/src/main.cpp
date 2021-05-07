#include <Arduino.h>
#include <RotaryEncoder.h>
#include <TB6612FNG.h>

Tb6612fng motors(27, 12, 13, 25, 32, 33, 26);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  motors.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0; i<360; i++) {
    float speed = sin((double)i*2*3.14/360.0);
    motors.drive(speed, -speed);
    delay(50);
  }
}