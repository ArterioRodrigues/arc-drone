#include "filter.h"
#include <cmath>

Filter::Filter(double alpha) {
    this->_alpha = alpha;
    this->_roll = 0.0;
    this->_pitch = 0.0;
}
std::pair<double, double> Filter::nextAngle(sensors_vec_t gyro, sensors_vec_t accel, double dt) {
    double rollAccel = atan2(accel.y, accel.z);
    double pitchAccel = atan2(-accel.x, accel.z);

    this->_roll = this->_alpha * (this->_roll + gyro.x * dt) + (1 - this->_alpha) * rollAccel;

    this->_pitch = this->_alpha * (this->_pitch + gyro.y * dt) + (1 - this->_alpha) * pitchAccel;

    return {_roll, _pitch};
}
