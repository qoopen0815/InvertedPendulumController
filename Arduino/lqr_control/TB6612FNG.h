#pragma once

#include <Arduino.h>
#include <Wire.h>

//TB6612FNG pin map
#define AIN1 7    // left motor
#define AIN2 6
#define BIN1 13   // right motor
#define BIN2 12
#define PWMA 9    // left motor pwm
#define PWMB 10   // right motor pwm
#define STBY 8    // standby

namespace utils {

class TB6612FNG
{
private:

public:
  TB6612FNG() = default;
  ~TB6612FNG() = default;

  // initialize
  void init();
  // Speed -255 ~ 255
  void move(const int& speed);
  void turn(const int& rpm);
};

} // utils
