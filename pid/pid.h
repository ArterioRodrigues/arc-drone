#pragma once

class PID {
public:
  PID(double Kp, double Ki, double Kd);

  double compute(double setpoint, double measure, double dt);

private:
  double _Kp;
  double _Ki;
  double _Kd;

  double _previouseError;
  double _Iterm;
};
