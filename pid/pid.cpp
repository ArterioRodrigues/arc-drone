#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, double iLimit, double outputLimit) {
  this->_Kp = Kp;
  this->_Ki = Ki;
  this->_Kd = Kd;

  this->_iLimit = iLimit;
  this->_outputLimit = outputLimit;

  reset();
}

void PID::reset() {
  this->_previousMeasure = 0;
  this->_Iterm = 0;
  this->_lastOutput = 0;
  this->_hasPreviousMeasure = false;
}

void PID::resetIntegral() {
  this->_Iterm = 0;
}

double PID::compute(double setpoint, double measure, double dt) {
  // Two loop passes inside the same microsecond give dt == 0; the comparison
  // also rejects NaN so a bad dt can never poison the output with inf/NaN.
  if (!(dt > 0.0)) { return _lastOutput; }

  double error = setpoint - measure;
  double p = _Kp * error;

  _Iterm = clamp(_Iterm + _Ki * error * dt, -_iLimit, _iLimit);

  // Derivative on the measurement, not on the error: a setpoint step would
  // otherwise produce a derivative kick straight into the motors.
  double d = 0.0;
  if (_hasPreviousMeasure) {
    d = -_Kd * (measure - _previousMeasure) / dt;
  }
  _previousMeasure = measure;
  _hasPreviousMeasure = true;

  _lastOutput = clamp(p + _Iterm + d, -_outputLimit, _outputLimit);
  return _lastOutput;
}
