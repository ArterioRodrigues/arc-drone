#pragma once
#include <Arduino.h>
#include <Preferences.h>

// Persistent flight recorder.
//
// The numbers needed to tune this airframe - hover throttle, and the attitude it
// actually holds in the air - can only be measured in flight, which is exactly
// when there is no serial cable attached. Telemetry solves the bench case and
// nothing else.
//
// So the last flight is written to the ESP32's NVS flash and printed at the next
// boot: fly, land, kill, carry it back to the bench, plug in, reset, read the
// line. NVS is used rather than RAM because the interesting case includes
// pulling the battery, and rather than an SD card because there is no card.
//
// The write happens once per flight, on the kill transition, and never in the
// control path: an NVS commit takes milliseconds and erases a flash page, so
// doing it per pass would both stall the loop and wear the part out.

// Smoothing constant for the recorded attitude, in seconds.
//
// A single instantaneous sample at the moment of kill would catch whatever gust
// or stick input happened to be in progress. A few seconds of averaging is what
// separates a steady lean from a transient one, and it is the steady lean that
// diagnoses a false level reference or an untrimmed disturbance.
//
// It doubles as a trailing window, which is the reason it is short. The average
// runs from the moment throttle is advanced, so it also sees the craft sitting
// level on the ground during the throttle ramp; a long constant would let that
// ground time dilute the airborne lean towards zero - reporting "level" for a
// craft that was leaning, which is the exact wrong answer here. A short constant
// forgets the ground phase, so the recorded value reflects the last few seconds
// before the kill.
//
// It must stay short relative to how long the craft actually hovers. At 3.0 a
// three-second hover records only 63% of the true lean; at 1.5 it records 86%,
// and 96% by five seconds. Err short.
#define FLIGHT_AVG_TAU 1.5

class FlightRecorder {
public:
  FlightRecorder(double idleBase, double tau = FLIGHT_AVG_TAU);

  // Opens the NVS namespace. Call before printLast().
  void setup();

  // Prints the previous flight's record, or a notice if there is none.
  void printLast();

  // Clears accumulated state so each flight is recorded independently rather
  // than carrying the previous one's peak throttle forward. Call on re-arm.
  void reset();

  // Records that throttle has been advanced, which is what marks a flight as
  // worth saving, and tracks the peak.
  void noteThrottle(double base);

  // Feeds the attitude average. Safe to call every control pass; it is a couple
  // of multiplies and touches no flash.
  //
  // Call this ONLY while the craft is airborne. The interesting quantity is the
  // lean it holds in the air, and time spent sitting on the ground reads as
  // level - so including the throttle ramp drags the average toward zero by an
  // amount that depends on how long the ramp took, which makes two flights
  // incomparable.
  void update(double roll, double pitch, double dt);

  // Commits the record. A no-op unless throttle was advanced since the last
  // reset, so it is safe to call unconditionally from a kill branch that runs
  // every pass. Call AFTER the motors have been commanded to zero - nothing may
  // sit between the kill and the zero frame.
  void save(double killBase);

private:
  Preferences _prefs;

  double _idleBase;
  double _tau;

  double _maxBase;
  double _rollAvg;
  double _pitchAvg;
  // Seconds of airborne averaging behind the figures above. Recorded because
  // the average needs roughly 3x FLIGHT_AVG_TAU to settle, so without it there
  // is no way to tell a converged reading from one still climbing out of zero.
  double _airborneS;
  bool _pending;
};
