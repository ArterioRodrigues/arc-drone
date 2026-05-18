#include "pid.h"
#include <algorithm>

PID::PID(double Kp, double Ki, double Kd) {
  this->_Kp = Kp;
  this->_Ki = Ki;
  this->_Kd = Kd;

  this->_previouseError = 0;
  this->_Iterm = 0;
}

double PID::compute(double setpoint, double measure, double dt) {
  double error = setpoint - measure;
  double p = _Kp * error;
  double i = _Ki * error * dt;
  double d = _Kd * (error - _previouseError) / dt;

  _previouseError = error;
  _Iterm = std::clamp(_Iterm + i, -200.0, 200.0);

  return p + _Iterm + d;
}
