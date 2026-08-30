#pragma once

#define MOTOR_MIN 48.0
#define MOTOR_MAX 2047.0

struct Motors {
    int m1;
    int m2;
    int m3;
    int m4;
};

class Mixer {
  public:
    Motors compute(double base, double roll, double pitch, double yaw);
};
