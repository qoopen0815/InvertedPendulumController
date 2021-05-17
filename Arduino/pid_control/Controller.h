#pragma once

#include <Arduino.h>

namespace control {

struct Bound {
  double upper;
  double lower;
};

struct Parameter {
  double kp;
  double ki;
  double kd;
};

class PIDControl
{
public:
  PIDControl(const struct Parameter& param, const struct Bound& bound, const double& time_step);
  ~PIDControl() = default;
  
  const double& calculate(const double& ref, const double& cur);

  const Parameter& parameter(void) const { return param_; };
  void parameter(const Parameter& param) { param_ = param; };

private:
  struct Parameter param_;
  struct Bound bound_;
  double time_step_;
  double err_[3];
  double control_input_[2];
};

} // control
