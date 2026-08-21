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
// button ramps base at loop rate. 5 units per 50 ms gives a predictable
// 100 units/sec, which also makes hover throttle measurable without a serial
// cable: it is IDLE_BASE + 100 * seconds_held_to_lift_off.
const double THROTTLE_STEP = 5.0;
const unsigned long THROTTLE_REPEAT_MS = 50;
unsigned long lastThrottleMs = 0;

// Throttle above which the integrators are allowed to accumulate - in effect a
// "probably airborne" test. Throttle is the only signal available for this:
// there is no altitude sensor and no weight-on-wheels switch.
//
// It matters because the integrator's job is to trim a steady attitude error,
// and while the craft is sitting on the ground it CANNOT correct its attitude
// no matter what the motors do. Every second spent grounded above this gate
// therefore winds the integral against an error it is powerless to fix, and
// that stored offset is dumped into the motors as a bank the instant the craft
// leaves the ground.
//
// Gating at IDLE_BASE is far too low to do that job. Measured hover is ~450 and
// the ramp above is 100 units/sec, so an IDLE_BASE gate leaves ~3.5 seconds of
// grounded wind-up before liftoff. Sitting just under hover cuts that to well
// under a second.
//
// Re-measure this if the airframe's weight changes: it must stay just BELOW
// hover throttle. Set too high the integrator never runs and steady drift comes
// back; set too low the ground wind-up returns.
const double INTEGRAL_ENABLE_BASE = 400;

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
#define TELEMETRY_ENABLED 0 

// Angles are in radians, so these gains are per-radian: a 10 degree tilt is only
// 0.175 rad, so Kp 200 produces 35 DShot units of correction.
//
// Roll and pitch are both live and both bench-verified (steps 1-4 below). Pitch
// deliberately runs lower Kp/Kd than roll: those are the numbers that produced a
// stable hover, and there is no reason to raise them to match roll until pitch
// actually misbehaves.
//
// Ki is now live. It was held at 0 until the craft could hold a clean hover,
// which it now does. Its job is to trim a STEADY error, and at Ki 0 the loop
// cannot null one - P simply settles wherever it balances, holding a small
// permanent bank. That bank is not a stability problem, it is a drift problem: a
// 3 degree lean is about 0.5 m/s^2 sideways, which builds into metres per second
// within a few seconds, and with no position sensor nothing ever pulls it back.
// Ki is what removes the lean itself.
//
// Ki is in DShot units per (radian-second) and is INDEPENDENT of Kp - the I term
// is added straight to the output - so the same value gives roll and pitch equal
// trim authority even though their Kp differ. 50 reaches the ~10 units needed to
// cancel a 3 degree bank in roughly 4 seconds. Raise it if drift persists, lower
// it if the craft develops a slow wallow.
//
// Ki depends on the ground wind-up gate above (INTEGRAL_ENABLE_BASE) to be safe.
// Without it the integral accumulates during the throttle ramp while the craft
// is still on the ground and unable to correct, then dumps that offset as a bank
// at liftoff.
//
// Kd is damping and is what stops an overshoot turning into a divergent
// oscillation. Do not test with Kd 0.
PID rollPid(200, 50, 10);
PID pitchPid(100, 50, 5);

// Yaw is a RATE controller, not an angle controller: compute() is fed gyro.z as
// the measurement, so Kp acts as rate damping - it resists rotation rather than
// holding a heading. Its Kp is therefore per rad/s, NOT per radian like roll and
// pitch, so the roll/pitch numbers are not comparable and must not be copied
// across: 200 here would be an enormous gain, not an equivalent one. Start at
// about 30.
//
// Kd stays 0 because there is no yaw acceleration signal to feed it - compute()
// is passed 0 for measureRate, which pid.h requires exactly when Kd is 0.
//
// Ki stays 0 for now, but not because a magnetometer is missing: integrating
// yaw RATE error gives heading relative to where the craft started, which is how
// heading-hold works on most flight controllers without a compass. It is left at
// 0 because it drifts with gyro bias and because plain rate damping should be
// proven first.
//
// All three are 0 today: hover testing showed the craft holds its heading
// unaided, so the prop pairs are torque-balanced and there is nothing to
// correct. Before giving yaw any gain, run bench step 5 - the yaw sign is
// unverified, and an inverted yaw axis is positive feedback that spins up.
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
// accRoll and accPitch are the angles implied by the accelerometer alone. They
// are printed next to the filtered angles and the matching gyro axis
// specifically to check axis agreement on BOTH axes - see the bench procedure at
// the top of loop(). Printing accRoll without accPitch is what let a pitch axis
// fault stay hidden.
void printTelemetry(double roll, double pitch, double accRoll, double accPitch,
                    sensors_vec_t gyro, sensors_vec_t acceleration,
                    double rollResult, double pitchResult, double yawResult,
                    Motors motors) {
#if TELEMETRY_ENABLED
  Serial.printf("base=%6.1f | roll=%7.3f accRoll=%7.3f gyroX=%7.3f"
                " | pitch=%7.3f accPitch=%7.3f gyroY=%7.3f"
                " | gyro z=%7.2f"
                " | accel x=%6.2f y=%6.2f z=%6.2f"
                " | pid r=%7.1f p=%7.1f y=%7.1f"
                " | m1=%4d m2=%4d m3=%4d m4=%4d | dt=%.4f\n",
                base, roll, accRoll, gyro.x,
                pitch, accPitch, gyro.y,
                gyro.z,
                acceleration.x, acceleration.y, acceleration.z,
                rollResult, pitchResult, yawResult,
                motors.m1, motors.m2, motors.m3, motors.m4, dt);
#endif
}

// BENCH PROCEDURE - PROPS OFF. Do these in order; do not skip to step 6.
//
// Steps 1/2 cover roll, steps 3/4 cover pitch, step 5 covers yaw. Every axis
// needs checking before it is given a gain. A passing roll check says nothing
// about pitch: the roll check verifies the left/right motor pairing, and the
// left pair is m1/m3 and the right pair m2/m4 whichever end of the frame is the
// front. So a front/back swap - or a pitch sign fault anywhere else - sails
// through steps 1 and 2 untouched and only reveals itself in the air, as one end
// of the craft climbing away under power.
//
// 1. Roll axis agreement. Rock the frame slowly right and left and watch
//    telemetry. While rolling right, `roll` and `accRoll` must both increase AND
//    `gyroX` must be positive. If gyroX has the opposite sign to the direction
//    roll is moving, the IMU's gyro and accel disagree about which way is
//    positive. The complementary filter then fights itself and D becomes
//    anti-damping, which no amount of gain tuning or sign flipping can fix - the
//    craft will always diverge. Fix the mounting/axis mapping before going
//    further.
//
// 2. Roll motor direction. Hold at idle, tilt the frame right (right side down).
//    m2/m4 must speed up. If m1/m3 speed up instead, swap the left and right
//    motor pairs in your wiring - do not negate anything in software.
//
// 3. Pitch axis agreement. Same as step 1 on the other axis: tilt the nose down
//    slowly and `pitch` and `accPitch` must both increase together while `gyroY`
//    is positive. If `pitch` and `accPitch` move in opposite directions, or
//    `gyroY` opposes them, the pitch axis has the same self-fighting filter
//    problem described in step 1 and must be fixed at the mounting/axis mapping.
//
// 4. Pitch motor direction. At idle, tilt the nose DOWN: the BACK pair must
//    speed up. The mixer's back pair is m3/m4 (see pid/mixer.cpp), so if m3/m4
//    are the motors that speed up while the nose is down, and m3/m4 are
//    physically at the BACK, pitch is correct. If the motors that speed up are
//    the ones physically at the FRONT, the front/back wiring is swapped: swap
//    the m1/m2 leads with the m3/m4 leads so the frame matches the mixer's
//    layout - do not negate anything in software, for the reason given at the
//    top of pid/mixer.cpp.
//
// 5. Yaw motor direction. ONLY needed when yaw gains are non-zero - yawPid is
//    currently (0, 0, 0), so yaw contributes nothing and this check can wait.
//    Do it before ever giving yaw a gain, because an inverted yaw axis is
//    positive feedback: the correction adds to the rotation it should oppose
//    and the craft accelerates into a spin.
//
//    This check has no step-1 equivalent. Roll and pitch can be cross-checked
//    against the accelerometer because gravity gives an absolute reference for
//    tilt; there is no such reference for heading, so the motor-direction test
//    below is the only check available.
//
//    Temporarily set yawPid to (30, 0, 0). Note that 30 is not comparable to
//    the roll/pitch numbers: yaw is a RATE controller fed gyro.z, so its Kp is
//    per rad/s, while roll and pitch are ANGLE controllers with Kp per radian.
//    Copying 200 across would be a huge gain, not an equivalent one.
//
//    With props off, note which way each motor spins by watching the bell. Then
//    at idle rotate the frame nose-right (clockwise seen from above): the two
//    motors that speed up must be the CLOCKWISE-spinning pair. A clockwise prop
//    reacts against the frame counter-clockwise, which is what opposes the
//    rotation you applied. If the counter-clockwise pair speeds up instead, yaw
//    is backwards - fix it by correcting the prop/motor rotation directions so
//    the diagonals match the mixer's m1/m4 and m2/m3 pairing, not by negating
//    anything in software.
//
// 6. Only once the steps above all pass, put props on.
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

      if (base < INTEGRAL_ENABLE_BASE) {
        // Not flying yet, so the craft cannot level itself and the integrator
        // would wind to its limit and dump that offset into the motors the
        // moment it left the ground. P and D still run, so tilting the frame
        // gives an immediate, visible bench response.
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
        std::pair<double, double> accelAngles =
            Filter::anglesFromAccel(acceleration);
        printTelemetry(roll, pitch, accelAngles.first, accelAngles.second,
                       gyro, acceleration,
                       rollResult, pitchResult, yawResult, motors);
      }
  });
}


