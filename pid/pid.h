#pragma once

// std::clamp needs C++17, which the ESP32 Arduino core does not build with, so
// use a local inline helper. Unlike Arduino's constrain macro this evaluates
// its argument once.
inline double clamp(double value, double low, double high) {
  if (value < low) { return low; }
  if (value > high) { return high; }
  return value;
}

// Cap on the accumulated integral, in DShot units. Fixed rather than a
// constructor argument: it is a windup guard, not a tuning knob, and passing 0
// by mistake silently disables the I term.
//
// Sized against what P can command, because the I term is added straight to the
// output and so competes with P on equal footing. Roll's Kp is 200 per radian,
// so a 10 degree error is only 35 units - at the old limit of 200 the
// integrator could reach the equivalent of a 57 degree P error and bank the
// craft hard on its own, while P was still whispering. That is an authority
// limit dressed up as a windup guard.
//
// 40 keeps I at roughly the strength of P at a 10 degree lean: enough to trim
// out a steady bank, not enough to fly the craft somewhere P never asked it to
// go. Its job is to null a small residual error, so if it ever needs to be
// raised much beyond this, the real fault is upstream - a false level
// reference or a mis-scaled Kp - and raising the cap only hides it.
#define PID_INTEGRAL_LIMIT 40.0

class PID {
public:
  PID(double Kp, double Ki, double Kd);

  // measureRate is the measured rate of change of `measure` - feed the gyro
  // straight in. Differentiating `measure` here instead would amplify sensor
  // noise and inherit the complementary filter's lag, both of which weaken
  // damping, and weak damping is what shows up as oscillation. Pass 0 when
  // Kd is 0.
  //
  // The output is intentionally unbounded. The mixer already scales corrections
  // into the throttle headroom that actually exists, so a limit here would only
  // hide the effect of the gains.
  double compute(double setpoint, double measure, double measureRate, double dt);

  // Clears all accumulated state. Call on kill/re-arm.
  void reset();

  // Clears only the integral, so it can be called every pass while grounded.
  void resetIntegral();

private:
  double _Kp;
  double _Ki;
  double _Kd;

  double _Iterm;
  double _lastOutput;
};
