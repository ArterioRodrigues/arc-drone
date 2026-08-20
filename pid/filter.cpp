#include "filter.h"
#include <cmath>

Filter::Filter(double timeConstant) {
    this->_timeConstant = timeConstant;
    this->_roll = 0.0;
    this->_pitch = 0.0;
    this->_rollTrim = 0.0;
    this->_pitchTrim = 0.0;
}

std::pair<double, double> Filter::anglesFromAccel(sensors_vec_t accel) {
    return {atan2(accel.y, accel.z), atan2(-accel.x, accel.z)};
}

void Filter::setAngles(double roll, double pitch) {
    this->_roll = roll;
    this->_pitch = pitch;
}

void Filter::setLevelReference(double roll, double pitch) {
    this->_rollTrim = roll;
    this->_pitchTrim = pitch;
}

std::pair<double, double> Filter::nextAngle(sensors_vec_t gyro, sensors_vec_t accel, double dt) {
    if (!(dt > 0.0)) { return {_roll - _rollTrim, _pitch - _pitchTrim}; }

    std::pair<double, double> accelAngles = anglesFromAccel(accel);

    // Derive the blend from dt rather than using a fixed constant. With a fixed
    // alpha the filter's time constant is alpha*dt/(1-alpha), so it silently
    // changes with loop rate - at the rate this loop actually runs, an alpha of
    // 0.98 worked out to about 0.1s, short enough that the estimate chased the
    // accelerometer.
    double alpha = this->_timeConstant / (this->_timeConstant + dt);

    this->_roll = alpha * (this->_roll + gyro.x * dt) + (1 - alpha) * accelAngles.first;
    this->_pitch = alpha * (this->_pitch + gyro.y * dt) + (1 - alpha) * accelAngles.second;

    return {this->_roll - this->_rollTrim, this->_pitch - this->_pitchTrim};
}
