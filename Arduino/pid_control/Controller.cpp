#include "Controller.h"

namespace control {

PIDControl::PIDControl(const Parameter& param, const Bound& bound, const double& time_step)
: param_(param), bound_(bound), time_step_(time_step), 
  err_({0.0, 0.0, 0.0}), control_input_({0.0, 0.0})
{
}

const double& PIDControl::calculate(const double& ref, const double& cur)
{
  err_[2] = ((ref - cur)-err_[0])/time_step_;
  err_[0] = ref - cur;

  // calculate control input
  control_input_[0] = param_.kp*err_[0] + param_.ki*err_[1] + param_.kd*err_[2];

  // limit of control input
  control_input_[0] = min(control_input_[0], bound_.upper);
  control_input_[0] = max(control_input_[0], bound_.lower);

  // update
  control_input_[1] = control_input_[0];
  err_[1] += err_[0]*time_step_;

  // limit of i gain
  err_[1] = min(err_[1], bound_.upper);
  err_[1] = max(err_[1], bound_.lower);

  return control_input_[0];
}

} // control