#pragma once
#include <Arduino.h>
#include <Preferences.h>

// Pilot attitude trim.
//
// calibrateLevel() defines "level" as the attitude the craft was resting in at
// boot, which is only correct if the thrust axis was actually vertical at that
// moment. It usually is not: landing gear is rarely even, and an IMU glued on a
// couple of degrees off is normal. The controller then holds that error forever
// and the craft drifts in one direction - flying exactly as commanded, just
// commanded wrongly.
//
// This is the correction for that, and it is what every RC transmitter has had
// since the 1970s. It shifts the DEFINITION of level rather than fighting the
// consequences with gains, which is the only thing that can work: Ki nulls error
// against the reference and so cannot fix the reference itself.
//
// Held separately from the calibrated reference rather than folded into it, so
// that a re-calibration at the next boot cannot silently eat it, and so both
// numbers can be printed side by side - "reference -4.14 deg, trim +4.10 deg"
// tells the whole mechanical story at a glance.

// Degrees per button press.
//
// 0.25 gives roughly 16 taps to cover the 4 degrees this airframe needs, which
// is fine enough to stop on the right value and coarse enough to get there.
// Each step is only 0.25 deg * Kp 100 = 0.44 DShot units of instantaneous
// change, so a tap mid-hover produces no visible jolt.
#define TRIM_STEP_DEG 0.25

// Total authority, in degrees, per axis.
//
// A trim beyond this is not correcting a mounting error, it is commanding a
// bank - at 10 degrees the craft accelerates sideways at 1.7 m/s^2. The clamp
// exists so a stuck or spammed D-pad cannot fly the craft into a wall.
#define TRIM_LIMIT_DEG 10.0

class AttitudeTrim {
public:
  AttitudeTrim(double stepDeg = TRIM_STEP_DEG, double limitDeg = TRIM_LIMIT_DEG);

  // Opens NVS and loads the stored trim. Call before the first print.
  void setup();

  void printCurrent();

  // Applies +/-1 (or more) steps per axis. Returns true if anything actually
  // changed, so the caller can avoid re-pushing an unchanged value or logging
  // a press that hit the clamp.
  bool adjust(int rollSteps, int pitchSteps);

  // Commits to NVS. A no-op unless the trim changed since the last save, so it
  // is safe to call from a kill branch that runs every pass.
  //
  // Deliberately NOT called on the button press: an NVS commit takes
  // milliseconds and the press is handled inside the control path, so saving
  // per tap would stall the loop once per tap.
  void save();

  // Radians, for handing to the filter.
  double roll() const { return this->_roll; }
  double pitch() const { return this->_pitch; }

private:
  Preferences _prefs;

  double _step;   // radians
  double _limit;  // radians

  double _roll;   // radians
  double _pitch;  // radians

  bool _dirty;
};
