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
//
// Raised from 1.0 after this exact failure was observed in flight: the craft
// visibly leaned and flew off in one direction while the recorder logged an
// airborne lean of 0.02 deg. A leaning, accelerating quad has its thrust along
// body z, so the horizontal components the tilt would show up in largely cancel
// and the accelerometer reports LEVEL - plausibly, and with a magnitude near 1g
// that sails through the trust gate below. At 1.0 that false "level" won within
// about a second, so a real lean simply dissolved out of the estimate and the
// controller held the tilt believing it was flat.
//
// 3.0 is affordable because the gyro bias is calibrated at every boot: the
// residual after calibration is on the order of 0.001 rad/s, or about 0.3 deg
// over a 3-second window. That is the price, and it buys an estimate that
// survives the transient instead of being talked out of it.
//
// Do not raise this much further without a better vertical reference. Past a
// few seconds the bias term stops being negligible, and the accelerometer is
// still the ONLY thing anchoring this estimate to true vertical - a longer
// window delays accel corruption but also delays every genuine correction.
#define FILTER_TIME_CONSTANT 3.0

// Band around 1g within which an accelerometer sample is believed to be
// measuring gravity and nothing else.
//
// This matters more than it looks. The accelerometer is the ONLY thing
// anchoring this estimate to true vertical, so a corrupted sample does not
// merely add noise to the angle - it moves the definition of level, and the
// controller then banks the craft to match it. That failure is invisible from
// the outside: the loop is not failing to correct, it is correcting perfectly
// to a wrong target, which is why it never recovers.
//
// A sample whose magnitude is not close to gravity is measuring gravity plus
// something else - a vibration spike, a strike, a hard manoeuvre - so it is
// discarded and the estimate coasts on the gyro until a clean one arrives.
//
// Note the limit of this test: it catches TRANSIENT corruption. It cannot catch
// sustained vibration that rectifies into a DC offset while leaving the
// magnitude near 1g. Soft-mounting the IMU is the fix for that; this is not a
// substitute for it.
#define ACCEL_TRUST_LOW 0.8
#define ACCEL_TRUST_HIGH 1.2

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

  // Pilot trim, applied on top of the calibrated level reference.
  //
  // Held separately rather than folded into the reference because
  // setLevelReference() runs on every boot: fold them together and each
  // calibration silently discards the trim. Keeping them apart also means both
  // can be reported, and "reference -4.14 deg, trim +4.10 deg" is the reading
  // that identifies a mechanical mounting error rather than hiding it.
  void setPilotTrim(double roll, double pitch);

private:
  double _roll;
  double _pitch;
  double _rollTrim;
  double _pitchTrim;
  double _rollTrimPilot;
  double _pitchTrimPilot;
  double _timeConstant;
};
