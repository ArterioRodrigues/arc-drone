#pragma once

class PID {
public:
  PID(double Kp, double Ki, double Kd, double iLimit = 200.0,
      double outputLimit = 500.0);

  double compute(double setpoint, double measure, double dt);

  // Clears the integral/derivative state. Call whenever the motors are idle or
  // re-armed, otherwise the integrator keeps winding up while on the ground.
  void reset();

private:
  double _Kp;
  double _Ki;
  double _Kd;

  double _iLimit;
  double _outputLimit;

  double _previousMeasure;
  double _Iterm;
  double _lastOutput;
  bool _hasPreviousMeasure;
};
