#pragma once

// std::clamp needs C++17, which the ESP32 Arduino core does not build with, so
// use a local inline helper. Unlike Arduino's constrain macro this evaluates
// its argument once.
inline double clamp(double value, double low, double high) {
  if (value < low) { return low; }
  if (value > high) { return high; }
  return value;
}

class PID {
public:
  PID(double Kp, double Ki, double Kd, double iLimit = 200.0,
      double outputLimit = 500.0);

  double compute(double setpoint, double measure, double dt);

  // Preferred form. Takes the measured rate of change (e.g. the gyro reading)
  // instead of differentiating the measurement, which avoids amplifying sensor
  // noise and avoids the lag the complementary filter adds. Both hurt damping,
  // and poor damping is what shows up as oscillation.
  double compute(double setpoint, double measure, double measureRate, double dt);

  // Clears the integral/derivative state. Call whenever the motors are idle or
  // re-armed, otherwise the integrator keeps winding up while on the ground.
  void reset();

  // Clears only the accumulated integral. Unlike reset() this keeps the
  // derivative history, so it can be called every pass while grounded without
  // destroying the D term.
  void resetIntegral();

private:
  double computeWithRate(double setpoint, double measure, double measureRate,
                         double dt);

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
