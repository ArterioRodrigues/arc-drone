#include "mixer.h"
#include <algorithm>
Motors Mixer::compute(double base, double roll, double pitch, double yaw) {
  Motors motors;
  motors.m1 = constrain(base - roll + pitch + yaw, 48.0, 2047.0);
  motors.m2 = constrain(base + roll + pitch - yaw, 48.0, 2047.0);
  motors.m3 = constrain(base - roll - pitch - yaw, 48.0, 2047.0);
  motors.m4 = constrain(base + roll - pitch + yaw, 48.0, 2047.0);
 
  return motors;
}
