#include "filter.h"
#include <cmath>

Filter::Filter(double timeConstant) {
    this->_timeConstant = timeConstant;
    this->_roll = 0.0;
    this->_pitch = 0.0;
    this->_rollTrim = 0.0;
    this->_pitchTrim = 0.0;
    this->_rollTrimPilot = 0.0;
    this->_pitchTrimPilot = 0.0;
}

// Roll is exact at any attitude: atan2(y, z) reduces to the roll angle no matter
// what pitch is doing, because both components carry the same cos(pitch) factor
// and the ratio cancels it.
//
// Pitch does NOT have that luxury. The obvious mirror of the roll expression,
// atan2(-x, z), is only correct while roll is zero - z shrinks with roll, so the
// denominator collapses and the reported pitch is inflated. At 30 deg of roll a
// true 30 deg pitch reads 33.7; at 60 deg of roll a true 20 deg pitch reads 36.
// The full horizontal magnitude sqrt(y^2 + z^2) is what removes the roll
// dependence.
//
// The error only appears when BOTH axes are tilted, which is why a pure roll or
// pure pitch bench test passes and the fault only shows up in combined motion -
// as a phantom pitch error the controller then dutifully corrects for.
std::pair<double, double> Filter::anglesFromAccel(sensors_vec_t accel) {
    return {atan2(accel.y, accel.z), atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z))};
}

void Filter::setAngles(double roll, double pitch) {
    this->_roll = roll;
    this->_pitch = pitch;
}

void Filter::setLevelReference(double roll, double pitch) {
    this->_rollTrim = roll;
    this->_pitchTrim = pitch;
}

void Filter::setPilotTrim(double roll, double pitch) {
    this->_rollTrimPilot = roll;
    this->_pitchTrimPilot = pitch;
}

std::pair<double, double> Filter::nextAngle(sensors_vec_t gyro, sensors_vec_t accel, double dt) {
    if (!(dt > 0.0)) {
        return {_roll - _rollTrim - _rollTrimPilot, _pitch - _pitchTrim - _pitchTrimPilot};
    }

    double magnitude = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    bool trustAccel =
        magnitude > ACCEL_TRUST_LOW * SENSORS_GRAVITY_EARTH && magnitude < ACCEL_TRUST_HIGH * SENSORS_GRAVITY_EARTH;

    std::pair<double, double> accelAngles = anglesFromAccel(accel);

    // Derive the blend from dt rather than using a fixed constant. With a fixed
    // alpha the filter's time constant is alpha*dt/(1-alpha), so it silently
    // changes with loop rate - at the rate this loop actually runs, an alpha of
    // 0.98 worked out to about 0.1s, short enough that the estimate chased the
    // accelerometer.
    //
    // alpha of exactly 1 drops the accelerometer term entirely, leaving pure
    // gyro integration for this pass. Safe for a handful of passes; if the
    // accelerometer were rejected indefinitely the estimate would drift away
    // with gyro bias and nothing would pull it back.
    double alpha = trustAccel ? this->_timeConstant / (this->_timeConstant + dt) : 1.0;

    this->_roll = alpha * (this->_roll + gyro.x * dt) + (1 - alpha) * accelAngles.first;
    this->_pitch = alpha * (this->_pitch + gyro.y * dt) + (1 - alpha) * accelAngles.second;

    return {this->_roll - this->_rollTrim - this->_rollTrimPilot,
            this->_pitch - this->_pitchTrim - this->_pitchTrimPilot};
}
