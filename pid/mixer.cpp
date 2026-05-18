#include "mixer.h"
#include <algorithm>
Motors Mixer::compute(double base, double roll, double pitch, double yaw) {
  Motors motors;
  motors.m1 = std::clamp(base - roll - pitch + yaw, 48.0, 2047.0);
  motors.m2 = std::clamp(base + roll - pitch - yaw, 48.0, 2047.0);
  motors.m3 = std::clamp(base - roll + pitch - yaw, 48.0, 2047.0);
  motors.m4 = std::clamp(base + roll + pitch + yaw, 48.0, 2047.0);

  return motors;
}
