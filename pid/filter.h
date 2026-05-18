#pragma once
#include <utility>


class Filter {
public:
  Filter(double alpha = 0.98);
  std::pair<double, double> nextAngle(sensors_vec_t gyro, sensors_vec_t accel, double dt);

private:
  double _roll;
  double _pitch;
  double _alpha;
};
