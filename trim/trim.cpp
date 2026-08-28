#include "trim.h"

#define TRIM_NAMESPACE "arctrim"
#define KEY_ROLL "rollRad"
#define KEY_PITCH "pitchRad"

static double clampTrim(double value, double limit) {
  if (value < -limit) { return -limit; }
  if (value > limit) { return limit; }
  return value;
}

AttitudeTrim::AttitudeTrim(double stepDeg, double limitDeg) {
  this->_step = stepDeg * PI / 180.0;
  this->_limit = limitDeg * PI / 180.0;
  this->_roll = 0;
  this->_pitch = 0;
  this->_dirty = false;
}

void AttitudeTrim::setup() {
  this->_prefs.begin(TRIM_NAMESPACE, false);

  // Re-clamped on load rather than trusted: a stored value from an older build
  // with a wider limit would otherwise come back and fly the craft sideways.
  this->_roll = clampTrim(this->_prefs.getFloat(KEY_ROLL, 0), this->_limit);
  this->_pitch = clampTrim(this->_prefs.getFloat(KEY_PITCH, 0), this->_limit);
  this->_dirty = false;
}

void AttitudeTrim::printCurrent() {
  Serial.printf("Pilot trim: roll=%+.2f deg pitch=%+.2f deg\n",
                this->_roll * 180.0 / PI, this->_pitch * 180.0 / PI);
}

bool AttitudeTrim::adjust(int rollSteps, int pitchSteps) {
  if (rollSteps == 0 && pitchSteps == 0) { return false; }

  double newRoll = clampTrim(this->_roll + rollSteps * this->_step, this->_limit);
  double newPitch = clampTrim(this->_pitch + pitchSteps * this->_step, this->_limit);

  // A press that lands on the clamp changes nothing, and reporting it as a
  // change would print a stream of identical trim lines while the stick is held
  // against the limit.
  if (newRoll == this->_roll && newPitch == this->_pitch) { return false; }

  this->_roll = newRoll;
  this->_pitch = newPitch;
  this->_dirty = true;
  return true;
}

void AttitudeTrim::save() {
  if (!this->_dirty) { return; }
  this->_dirty = false;

  this->_prefs.putFloat(KEY_ROLL, (float)this->_roll);
  this->_prefs.putFloat(KEY_PITCH, (float)this->_pitch);

  Serial.printf("Trim saved: roll=%+.2f deg pitch=%+.2f deg\n",
                this->_roll * 180.0 / PI, this->_pitch * 180.0 / PI);
}
