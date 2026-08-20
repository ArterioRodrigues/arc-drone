#include "esc/esc.h"
#include "esc/esc.cpp"
#include "controller/controller.h"
#include "controller/controller.cpp"
#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.cpp"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include "pid/filter.h"
#include "pid/filter.cpp"
#include "pid/mixer.h"
#include "pid/mixer.cpp"

Drone::Controller controller;
MPU6050 mpu6050; //Default to GPIO 21 (SDA) and 22 (SCL)
ESC esc(DSHOT::DSHOT300);
                 
double dt = 0;

// Throttle trim range. base starts at idle and the pilot walks it up with Cross
// / down with Circle.
const double IDLE_BASE = 100;
const double MAX_BASE = 1600;

// The button handler runs every loop pass, so without a repeat interval a held
// button ramps base at loop rate. 1 unit per 50 ms gives a predictable
// 20 units/sec, which also makes hover throttle measurable without a serial
// cable: it is IDLE_BASE + 20 * seconds_held_to_lift_off.
const double THROTTLE_STEP = 5.0;
const unsigned long THROTTLE_REPEAT_MS = 50;
unsigned long lastThrottleMs = 0;

double base = IDLE_BASE;
unsigned long lastTime = 0;

// Control loop period. The loop spins far faster than this, so the rate gate is
// what gives the filter and PID a steady, known dt - and dt is what the gyro
// integration and the integral term are scaled by, so it has to be real elapsed
// time, measured once per control iteration and nowhere else.
//
// 4 ms (250 Hz) matches the DShot300 frame interval, so ESC frames also land at
// the rate the protocol expects.
const unsigned long CONTROL_PERIOD_US = 4000;

// Serial is slow; printing every pass would stall the control loop.
const unsigned long TELEMETRY_INTERVAL_MS = 200;
unsigned long lastTelemetryMs = 0;

// Telemetry is off in flight. The line is ~185 characters, which at 115200 baud
// takes ~16 ms to push out - and Serial blocks once its TX buffer fills, so that
// stall lands directly in the control loop. Set to 1 for bench debugging.
//
// A compile-time switch rather than commented-out code: the body still has to
// compile, so it cannot rot and go stale while disabled.
#define TELEMETRY_ENABLED 1

// Angles are in radians, so these gains are per-radian: a 10 degree tilt is only
// 0.175 rad, so Kp 200 produces 35 DShot units of correction.
//
// Only roll is live. Pitch and yaw are Kp/Ki/Kd = 0 so the airframe cannot fight
// back on axes that are not being tested - isolate one axis, get it right, then
// copy the numbers across.
//
// Ki stays 0 until roll holds a clean hover. It exists to trim steady drift, and
// drift is acceptable right now while stability is not. At Ki 0 the loop cannot
// null a steady disturbance - P settles wherever it balances, holding a small
// bank. That is expected and tolerated for now.
//
// Kd is damping and is what stops an overshoot turning into a divergent
// oscillation. Do not test with Kd 0.
PID rollPid(200, 0, 10);
PID pitchPid(0, 0, 0);

// Yaw is a RATE controller, not an angle controller: compute() is fed gyro.z as
// the measurement, so Kp acts as rate damping - it resists rotation rather than
// holding a heading. Ki must stay 0: there is no magnetometer, so there is no
// absolute heading for an integrator to work against.
PID yawPid(0, 0, 0);

Filter filter;
Mixer mixer;

// Arduino's auto-generated prototypes do not carry default arguments, so
// declare this explicitly.
void calibrateLevel(uint16_t samples = 500);

// Latched by Triangle. Survives a controller dropout on purpose: once killed,
// nothing spins again until the pilot re-arms with the combo below.
bool killed = false;


void setup(void) {
  Serial.begin(115200);
  controller.setup();
  mpu6050.setup();

  // Must happen before the props can spin and while the craft is stationary and
  // level. Everything downstream measures attitude relative to what is captured
  // here, so a craft calibrated while tilted will hold that tilt in flight.
  mpu6050.calibrateGyro();
  calibrateLevel();

  esc.setup();
  lastTime = micros();
}

// Averages the resting accelerometer to establish which way is down, then tells
// the filter that this attitude is level. Without it an IMU mounted a couple of
// degrees off makes the controller hold a constant tilt, and the craft drifts
// steadily in one direction - it is flying exactly as commanded, just commanded
// wrongly.
void calibrateLevel(uint16_t samples) {
  Serial.println("Measuring level reference - keep the craft still and level...");

  double sumX = 0, sumY = 0, sumZ = 0;
  for (uint16_t i = 0; i < samples; i++) {
    mpu6050.read();
    sensors_vec_t accel = mpu6050.lastAcceleration();
    sumX += accel.x;
    sumY += accel.y;
    sumZ += accel.z;
    delay(2);
  }

  sensors_vec_t average;
  average.x = sumX / samples;
  average.y = sumY / samples;
  average.z = sumZ / samples;

  std::pair<double, double> level = Filter::anglesFromAccel(average);
  filter.setLevelReference(level.first, level.second);
  filter.setAngles(level.first, level.second);

  Serial.printf("Level reference: roll=%.2f deg pitch=%.2f deg\n",
                level.first * 180.0 / PI, level.second * 180.0 / PI);
}

// All periodic serial output lives here so it can be switched off in one place.
// Disabled it costs nothing; enabled it blocks the control loop for milliseconds
// at a time, which shows up as sluggish, late corrections.
//
// accRoll is the roll implied by the accelerometer alone. It is printed next to
// the filtered roll and gyro.x specifically to check axis agreement - see the
// bench procedure at the top of loop().
void printTelemetry(double roll, double pitch, double accRoll,
                    sensors_vec_t gyro, sensors_vec_t acceleration,
                    double rollResult, double pitchResult, double yawResult,
                    Motors motors) {
#if TELEMETRY_ENABLED
  Serial.printf("base=%6.1f | roll=%7.3f accRoll=%7.3f gyroX=%7.3f | pitch=%7.3f"
                " | gyro y=%7.2f z=%7.2f"
                " | accel x=%6.2f y=%6.2f z=%6.2f"
                " | pid r=%7.1f p=%7.1f y=%7.1f"
                " | m1=%4d m2=%4d m3=%4d m4=%4d | dt=%.4f\n",
                base, roll, accRoll, gyro.x, pitch,
                gyro.y, gyro.z,
                acceleration.x, acceleration.y, acceleration.z,
                rollResult, pitchResult, yawResult,
                motors.m1, motors.m2, motors.m3, motors.m4, dt);
#endif
}

// BENCH PROCEDURE - PROPS OFF. Do these in order; do not skip to step 3.
//
// 1. Axis agreement. Rock the frame slowly right and left and watch telemetry.
//    While rolling right, `roll` and `accRoll` must both increase AND `gyroX`
//    must be positive. If gyroX has the opposite sign to the direction roll is
//    moving, the IMU's gyro and accel disagree about which way is positive. The
//    complementary filter then fights itself and D becomes anti-damping, which
//    no amount of gain tuning or sign flipping can fix - the craft will always
//    diverge. Fix the mounting/axis mapping before going further.
//
// 2. Motor direction. Hold at idle, tilt the frame right (right side down).
//    m2/m4 must speed up. If m1/m3 speed up instead, swap the left and right
//    motor pairs in your wiring - do not negate anything in software.
//
// 3. Only once 1 and 2 both pass, put props on.
void loop() {
  controller.processController([]() {
    ControllerPtr ctl = controller.getController();
    bool connected = (ctl != nullptr && ctl->isConnected());

    if (!connected) {
      // Without a controller nothing below drives the ESCs, and they would time
      // out and disarm while we wait for a connection. Keep pinging them.
      esc.keepAlive();
      // Stale timestamp would otherwise produce a bogus dt on the first frame
      // after the controller shows up.
      lastTime = micros();
      return;
    }

    // Gamepad input is only sampled when a fresh HID report has arrived.
    // hasData() is a one-shot flag - arduino_get_controller_data() returns
    // NO_DATA and clears data_updated on read - so it is true only on the pass
    // that consumes a new packet.
    //
    // Reading buttons here is correct. Gating the CONTROL LOOP on it was not:
    // the flight code then ran at gamepad report rate, and because the
    // early-return path refreshed lastTime every pass, dt measured the gap
    // between two no-data passes (microseconds) rather than the real time since
    // the last filter update (milliseconds). The gyro term is gyro.x * dt, so a
    // dt that small contributed almost nothing and the estimate was left to the
    // accelerometer's 1-second path - the gyro read correctly but the angle
    // crawled, and the loop flew on second-old attitude.
    if (ctl->hasData()) {
      if (ctl->y() && !killed) {  // Triangle
        killed = true;
        Serial.println("KILLED - press Square + L1 to re-arm");
      }

      if (killed) {
        // Two-button combo to re-arm: a single stray press must not put the
        // props back to idle spin.
        if (ctl->x() && (ctl->buttons() & BUTTON_SHOULDER_L)) {
          killed = false;
          Serial.println("RE-ARMED");
        }
      } else {
        unsigned long nowMs = millis();
        if (nowMs - lastThrottleMs >= THROTTLE_REPEAT_MS) {
          if (ctl->a()) {  // Cross
            base = constrain(base + THROTTLE_STEP, IDLE_BASE, MAX_BASE);
            lastThrottleMs = nowMs;
          } else if (ctl->b()) {  // Circle
            base = constrain(base - THROTTLE_STEP, IDLE_BASE, MAX_BASE);
            lastThrottleMs = nowMs;
          }
        }
      }
    }

    // Everything past this point runs at a fixed rate, independent of the
    // gamepad. This is also what keeps DShot frames at the protocol's rate -
    // the loop itself spins far faster than the ESCs can be commanded.
    unsigned long now = micros();
    if (now - lastTime < CONTROL_PERIOD_US) { return; }
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    // A dropped or reconnecting controller can still leave a large gap; capping
    // dt stops one late frame from dumping a giant step into the integrator.
    dt = constrain(dt, 0.0, 0.05);

    if (killed) {
      // Hard stop: zero throttle straight to the ESCs, bypassing base/mixer so
      // no PID output can leak through. Still a real DShot frame, so the ESCs
      // stay armed and responsive for the re-arm.
      esc.sendDShotPacket(0, 0, 0, 0);
      base = IDLE_BASE;
      rollPid.reset();
      pitchPid.reset();
      yawPid.reset();
      return;
    }

      // One I2C read per pass, so accel and gyro come from the same instant.
      mpu6050.read();
      sensors_vec_t acceleration = mpu6050.lastAcceleration();
      sensors_vec_t gyro = mpu6050.lastGyro();

      std::pair<double, double> pair = filter.nextAngle(gyro, acceleration, dt);
      double roll = pair.first;
      double pitch = pair.second;

      if (base <= IDLE_BASE) {
        // Not commanded to fly, so the craft cannot level itself and the
        // integrator would wind to its limit and dump that offset into the
        // motors the moment throttle came up. P and D still run, so tilting the
        // frame gives an immediate, visible bench response.
        rollPid.resetIntegral();
        pitchPid.resetIntegral();
        yawPid.resetIntegral();
      }

      // Feed the gyro straight in as the derivative term. Differentiating the
      // filtered angle instead amplifies sensor noise and inherits the
      // complementary filter's lag, both of which weaken damping - and weak
      // damping is what shows up as oscillation.
      //
      // No correction factors here on purpose: the mixer owns the signs.
      double rollResult  = rollPid.compute(0, roll, gyro.x, dt);
      double pitchResult = pitchPid.compute(0, pitch, gyro.y, dt);
      double yawResult   = yawPid.compute(0, gyro.z, 0, dt);

      Motors motors = mixer.compute(base, rollResult, pitchResult, yawResult);
      esc.sendDShotPacket(motors.m1, motors.m2, motors.m3, motors.m4);

      unsigned long telemetryMs = millis();
      if (telemetryMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = telemetryMs;
        printTelemetry(roll, pitch, Filter::anglesFromAccel(acceleration).first,
                       gyro, acceleration,
                       rollResult, pitchResult, yawResult, motors);
      }
  });
}


