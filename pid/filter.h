#pragma once
#include <Adafruit_Sensor.h>
#include <utility>

// Seconds. How long the estimate leans on integrated gyro before the
// accelerometer pulls it back to vertical.
//
// The accelerometer only reads true gravity while the craft is not
// accelerating. In flight it reads thrust plus gravity, so during any
// translation it tilts and reports a lean the craft does not have. A short time
// constant lets that corrupt the estimate, the controller "corrects" a tilt that
// is not there, and the craft accelerates away in one direction. Long enough to
// ride out those transients, short enough that gyro drift cannot accumulate.
#define FILTER_TIME_CONSTANT 1.0

class Filter {
public:
  Filter(double timeConstant = FILTER_TIME_CONSTANT);

  std::pair<double, double> nextAngle(sensors_vec_t gyro, sensors_vec_t accel, double dt);

  // Roll and pitch implied by an accelerometer reading alone. Only meaningful
  // when the craft is stationary, since it assumes the measured acceleration is
  // purely gravity.
  static std::pair<double, double> anglesFromAccel(sensors_vec_t accel);

  // Sets the estimate directly. Used at startup so the loop does not have to
  // converge from an assumed zero.
  void setAngles(double roll, double pitch);

  // Records the attitude the craft rests at as its definition of level. An IMU
  // mounted a couple of degrees off would otherwise make the controller hold
  // that tilt, so the craft accelerates steadily in one direction.
  void setLevelReference(double roll, double pitch);

private:
  double _roll;
  double _pitch;
  double _rollTrim;
  double _pitchTrim;
  double _timeConstant;
};
