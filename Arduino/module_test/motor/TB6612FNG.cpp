#include "TB6612FNG.h"

namespace utils {
void TB6612FNG::init(){
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  digitalWrite(STBY, HIGH); // スタンバイ
}

void TB6612FNG::move(const int& speed){
  bool pin1;
  bool pin2;
  if (speed > 0){
    pin1 = 0x1;   // HIGH
    pin2 = 0x0;   // LOW
  }
  else{
    pin1 = 0x0;   // LOW
    pin2 = 0x1;   // HIGH
  }
  digitalWrite(AIN1, pin1);
  digitalWrite(AIN2, pin2);
  analogWrite(PWMA, abs(speed));
  digitalWrite(BIN1, pin1);
  digitalWrite(BIN2, pin2);
  analogWrite(PWMB, abs(speed));
}

void TB6612FNG::turn(const int& rpm){
  bool pin1;
  bool pin2;
  if (rpm > 0){
    pin1 = 0x1;   // HIGH
    pin2 = 0x1;   // HIGH
  }
  else{
    pin1 = 0x0;   // LOW
    pin2 = 0x0;   // LOW
  }
  digitalWrite(AIN1, pin1);
  digitalWrite(AIN2, pin2);
  analogWrite(PWMA, abs(rpm));
  digitalWrite(BIN1, pin1);
  digitalWrite(BIN2, pin2);
  analogWrite(PWMB, abs(rpm));
}

} // utils
