#include <Wire.h>
#include "TB6612FNG.h"

utils::TB6612FNG motor;

void setup()
{
  Serial.begin(115200);
  while (!Serial){}

  Wire.begin();
  motor.init();

  delay(100);
}

void loop()
{
  motor.move(100);
}
