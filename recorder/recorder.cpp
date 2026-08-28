#include "recorder.h"

// NVS namespace and key names. Both are capped at 15 characters by the NVS API,
// so these are kept short deliberately rather than for brevity's sake.
#define RECORDER_NAMESPACE "arcdrone"
#define KEY_KILL_BASE "killBase"
#define KEY_MAX_BASE "maxBase"
#define KEY_ROLL_DEG "rollDeg"
#define KEY_PITCH_DEG "pitchDeg"
#define KEY_AIRBORNE_S "airborneS"

FlightRecorder::FlightRecorder(double idleBase, double tau) {
  this->_idleBase = idleBase;
  this->_tau = tau;
  reset();
}

void FlightRecorder::setup() {
  // false = read/write. The namespace is created on first write.
  this->_prefs.begin(RECORDER_NAMESPACE, false);
}

void FlightRecorder::reset() {
  this->_maxBase = this->_idleBase;
  this->_rollAvg = 0;
  this->_pitchAvg = 0;
  this->_airborneS = 0;
  this->_pending = false;
}

void FlightRecorder::printLast() {
  if (!this->_prefs.isKey(KEY_MAX_BASE)) {
    Serial.println("No previous flight recorded.");
    return;
  }

  float airborneS = this->_prefs.getFloat(KEY_AIRBORNE_S, 0);

  Serial.printf("Last flight: base at kill=%.0f  max base=%.0f"
                "  | airborne %.1fs: roll=%.2f deg pitch=%.2f deg%s\n",
                this->_prefs.getFloat(KEY_KILL_BASE, 0),
                this->_prefs.getFloat(KEY_MAX_BASE, 0),
                airborneS,
                this->_prefs.getFloat(KEY_ROLL_DEG, 0),
                this->_prefs.getFloat(KEY_PITCH_DEG, 0),
                // The average needs ~3x the time constant to settle, so a short
                // hop understates the lean and must not be read as an improvement.
                airborneS < 3.0 * FLIGHT_AVG_TAU ? "  [SHORT - understated]" : "");
}

void FlightRecorder::noteThrottle(double base) {
  this->_pending = true;
  if (base > this->_maxBase) {
    this->_maxBase = base;
  }
}

void FlightRecorder::update(double roll, double pitch, double dt) {
  if (!(dt > 0.0)) {
    return;
  }

  // Same dt-derived blend as the attitude filter, for the same reason: a fixed
  // coefficient would silently change meaning with loop rate, while this pins
  // the average to a time constant in seconds.
  double alpha = dt / (this->_tau + dt);

  this->_rollAvg += (roll - this->_rollAvg) * alpha;
  this->_pitchAvg += (pitch - this->_pitchAvg) * alpha;
  this->_airborneS += dt;
}

void FlightRecorder::save(double killBase) {
  if (!this->_pending) {
    return;
  }
  this->_pending = false;

  double rollDeg = this->_rollAvg * 180.0 / PI;
  double pitchDeg = this->_pitchAvg * 180.0 / PI;

  this->_prefs.putFloat(KEY_KILL_BASE, (float)killBase);
  this->_prefs.putFloat(KEY_MAX_BASE, (float)this->_maxBase);
  this->_prefs.putFloat(KEY_ROLL_DEG, (float)rollDeg);
  this->_prefs.putFloat(KEY_PITCH_DEG, (float)pitchDeg);
  this->_prefs.putFloat(KEY_AIRBORNE_S, (float)this->_airborneS);

  Serial.printf("Flight recorded: base at kill=%.0f  max base=%.0f"
                "  airborne %.1fs  roll=%.2f deg pitch=%.2f deg\n",
                killBase, this->_maxBase, this->_airborneS, rollDeg, pitchDeg);
}
