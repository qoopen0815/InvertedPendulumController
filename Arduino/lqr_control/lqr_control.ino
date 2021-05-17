#include <RotaryEncoder.h>
#include <Ticker.h>
#include <Wire.h>

#include "Controller.h"
#include "Inclinometer.h"
#include "TB6612FNG.h"

#define PIN_IN1 2
#define PIN_IN2 3

#define TESTPINA 5
#define TESTPINB 11

void updateKalman();
void updateEncoder();
void controlMotor();

const double dt = 0.015;
const double rate = 1/dt;

utils::TB6612FNG motor;
Inclinometer inclinometer;
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

Ticker kalman_timer(updateKalman, 2500, 0, MICROS_MICROS); // 400Hz
Ticker encoder_timer(updateEncoder, 250, 0, MICROS_MICROS); // 4kHz
Ticker control_timer(controlMotor, 10); // 100Hz

control::Bound bound = {255.0, -255.0};

double theta_p[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
double theta_w[2] = {0.0, 0.0};  // 0: deg, 1: rad/s
double past_w;

double K[4] = {23.1167, 3.9229, 0.402, 1.2578};  // Feedback Gain K

void setup() {  
  Serial.begin(115200);
  Wire.begin();
  inclinometer.begin(rate);  
  motor.init();

  kalman_timer.start();
  encoder_timer.start();
  control_timer.start();

  past_w = 0.0;

  delay(100);
}

void loop() {
  kalman_timer.update();
  encoder_timer.update();
  control_timer.update();
}

void updateKalman() {
  inclinometer.updateTheta();
}

void updateEncoder() {
  encoder.tick();
}

void controlMotor() {
  theta_p[0] = (inclinometer.getFilteredTheta()-2.0) * 3.14/180.;
  theta_p[1] = inclinometer.getThetaDotFromGyro() * 3.14/180.;
  theta_w[0] = (double)encoder.getPosition() * 3.14/600.;
  theta_w[1] = (theta_w[0] - past_w)/dt;
  past_w = theta_w[0] - dt*theta_w[1];

  double control_input = theta_p[0]*K[0] + theta_p[1]*K[1] + theta_w[0]*K[2] + theta_w[1]*K[3];
  control_input = min(control_input, bound.upper);
  control_input = max(control_input, bound.lower);
  motor.move((int)(control_input*255.));
}
