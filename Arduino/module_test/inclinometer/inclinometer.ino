#include <Wire.h>
#include <Ticker.h>
#include "inclinometer.h"

#define TESTPINA 5
#define TESTPINB 11

void updateInclinometer();
void printFilteredTheta();

const double rate = 400.;
const double dt = 1./rate;

Inclinometer inclinometer;
Ticker kalman_timer(updateInclinometer, (int)(dt*1000000.), 0, MICROS_MICROS); // 400Hz
Ticker debug_timer(printFilteredTheta, 10); // 50Hz

void updateInclinometer() {
  digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
  inclinometer.updateTheta();
}
void printFilteredTheta() {
  digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
  Serial.println(inclinometer.getFilteredTheta());
}

void setup() {
  Serial.begin(115200);
  Serial.println("start");
  inclinometer.begin(rate);

  kalman_timer.start();
  debug_timer.start();
}

void loop() {
  kalman_timer.update();
  debug_timer.update();
}
