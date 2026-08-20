#include "pid.h"

PID::PID(double Kp, double Ki, double Kd) {
  this->_Kp = Kp;
  this->_Ki = Ki;
  this->_Kd = Kd;

  reset();
}

void PID::reset() {
  this->_Iterm = 0;
  this->_lastOutput = 0;
}

void PID::resetIntegral() { this->_Iterm = 0; }

double PID::compute(double setpoint, double measure, double measureRate, double dt) {
  // Two loop passes inside the same microsecond give dt == 0; the comparison
  // also rejects NaN so a bad dt can never poison the output with inf/NaN.
  if (!(dt > 0.0)) { return _lastOutput; }

  double error = setpoint - measure;

  _Iterm = clamp(_Iterm + _Ki * error * dt, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  // Derivative on the measurement, not on the error, so a setpoint change
  // cannot kick the motors.
  _lastOutput = _Kp * error + _Iterm - _Kd * measureRate;
  return _lastOutput;
}
