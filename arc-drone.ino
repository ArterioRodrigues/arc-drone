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
const double IDLE_BASE = 100;
// Throttle this airframe actually hovers at. Estimated at 800-1000, set to the
// low end deliberately: GROUND_BASE derives from this, and a value that is too
// high parks the threshold above real hover so the integrator never switches on
// and the craft keeps drifting.
//
// To measure it without a serial cable: the throttle trim is a known ramp of
// THROTTLE_STEP per THROTTLE_REPEAT_MS, currently 20 units/sec starting from
// IDLE_BASE. Hold Cross from idle and time it - hover throttle is
// IDLE_BASE + 20 * seconds_to_lift_off.
const double HOVER_BASE = 800;

// Ceiling for the throttle trim, set above hover so there is headroom to climb.
const double MAX_BASE = 1600;

// Authority ramps in over this range above idle, then stays full. This is only a
// soft start so corrections are not jerky just off idle - the mixer already
// scales corrections into whatever headroom the current throttle allows, so
// there is no need to keep suppressing them all the way up to hover.
const double FULL_AUTHORITY_BASE = 400;

// Correction authority never drops to zero, or the quad would stop responding
// entirely at bench throttle and there would be no way to verify the loop. This
// is the floor applied to the ramp.
const double MIN_AUTHORITY = 0.05;
// Below this the quad is treated as still on the ground, so the integrator is
// held clear to stop it winding up against a surface it cannot level itself off.
//
// Derived from HOVER_BASE so there is only one number to keep accurate. It has
// to sit just below hover, not down at idle: the throttle trim climbs at 20
// units/sec, so a threshold of 300 against a 1400 hover left the integrator
// accumulating for ~55s while still on the ground.
//
// In practice the exposure is small, because a craft sitting on flat ground that
// was calibrated on that same ground reports near-zero error and so integrates
// almost nothing. The real risk is taking off from a slope - do not.
const double GROUND_BASE = HOVER_BASE * 0.85;

// Bench-verified signs. The correct values depend on how the IMU is physically
// mounted, which cannot be determined from code - the mixer's own derivation
// assumes a standard orientation and is wrong for this airframe, so these are
// the authority. Verify props-off in bench mode (D-pad Up):
//   tilt right     -> m2/m4 must rise, else flip ROLL_SIGN
//   tilt nose down -> m1/m2 must rise, else flip PITCH_SIGN
//
// ROLL_SIGN is -1 because the bench test showed tilting left (m1/m3 side down)
// speeding up m2/m4 - the high side, which drives the craft further into the
// tilt instead of levelling it.
const double ROLL_SIGN = -1.0;
const double PITCH_SIGN = +1.0;

double base = IDLE_BASE;
double targetBase = IDLE_BASE;
unsigned long lastTime = 0;

// Throttle trim. The button handler runs every loop pass, so without a repeat
// interval a held button ramps base at loop rate (hundreds of steps a second).
// 1 unit per 50 ms gives a predictable 20 units/sec.
const double THROTTLE_STEP = 1.0;
const unsigned long THROTTLE_REPEAT_MS = 50;
unsigned long lastThrottleMs = 0;

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

// Angles are in radians, so these gains are per-radian. Kp 200 is the value the
// airframe last flew with; it was raised to 300 while chasing a weak response,
// but that turned out to be a sign problem, not a gain problem. Back to the
// known-good number - tune from here, one change at a time.
//
// Ki is staged at 0 on purpose. It exists to trim out steady drift, but drift is
// acceptable for now and stability is not - so it stays out of the loop until
// the craft holds a clean hover. Raise it to 50 (integral time Kp/Ki = 4s) once
// hover is stable and the only remaining complaint is a slow lean.
//
// For the record: at Ki 0 the loop cannot null a steady disturbance. P always
// stops short, because as the error shrinks so does the push, so it settles
// wherever it balances the disturbance and holds that small bank. That is
// expected, and it is the drift being tolerated for now.
PID rollPid(200, 0, 10, 50, 300);
PID pitchPid(200, 0, 10, 50, 300);

// Yaw is a RATE controller, not an angle controller: compute() is fed gyro.z
// directly as the measurement, so Kp here acts as rate damping - it resists
// rotation rather than holding a heading.
//
// It was 0, meaning yaw was completely unstabilised. Nothing opposed rotation
// about the vertical axis, while roll and pitch corrections actively induce it:
// motor torque scales with speed, so an asymmetric mix twists the frame. The
// craft yaws, keeps yawing, and spins in.
//
// Ki stays 0. There is no magnetometer, so there is no absolute heading to hold,
// and an integrator would wind up against a reference that does not exist.
PID yawPid(50, 0, 0, 50, 300);

Filter filter;
Mixer mixer;

// Arduino's auto-generated prototypes do not carry default arguments, so
// declare this explicitly.
void calibrateLevel(uint16_t samples = 500);

// Latched by Triangle. Survives a controller dropout on purpose: once killed,
// nothing spins again until the pilot re-arms with the combo below.
bool killed = false;

// Forces full correction authority regardless of throttle so the loop can be
// verified on the bench, where throttle is too low for a visible response.
// PROPS OFF.
bool benchMode = false;
bool lastDpadUp = false;


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
void printTelemetry(double authority, double roll, double pitch,
                    sensors_vec_t gyro, sensors_vec_t acceleration,
                    double rollResult, double pitchResult, double yawResult,
                    Motors motors) {
#if TELEMETRY_ENABLED
  Serial.printf("base=%6.1f auth=%.2f%s | roll=%7.2f pitch=%7.2f | gyro x=%7.2f y=%7.2f z=%7.2f"
                " | accel x=%6.2f y=%6.2f z=%6.2f"
                " | pid r=%7.1f p=%7.1f y=%7.1f"
                " | m1=%4d m2=%4d m3=%4d m4=%4d | dt=%.4f\n",
                base, authority, benchMode ? " BENCH" : "", roll, pitch,
                gyro.x, gyro.y, gyro.z,
                acceleration.x, acceleration.y, acceleration.z,
                rollResult, pitchResult, yawResult,
                motors.m1, motors.m2, motors.m3, motors.m4, dt);
#endif
}

void loop() {
  controller.processController([]() {
    ControllerPtr ctl = controller.getController();
    if(ctl == nullptr || !ctl->isConnected() || !ctl->hasData()) {
      // Without a controller the flight path below never runs, so nothing would
      // drive the ESCs and they would time out and disarm while we wait for a
      // connection. Keep pinging them with neutral throttle instead.
      esc.keepAlive();
      // Stale timestamp would otherwise produce a bogus dt on the first frame
      // after the controller shows up.
      lastTime = micros();
      return;
    }

    if(ctl->y() && !killed) {  // Triangle
      killed = true;
      Serial.println("KILLED - press Square + L1 to re-arm");
    }

    if(killed) {
      // Hard stop: zero throttle straight to the ESCs, bypassing base/mixer so
      // no PID output can leak through. Still a real DShot frame every loop, so
      // the ESCs stay armed and responsive for the re-arm.
      esc.sendDShotPacket(0, 0, 0, 0);
      base = IDLE_BASE;
      rollPid.reset();
      pitchPid.reset();
      yawPid.reset();
      lastTime = micros();

      // Two-button combo to re-arm: a single stray press must not put the props
      // back to idle spin.
      if(ctl->x() && (ctl->buttons() & BUTTON_SHOULDER_L)) {
        killed = false;
        Serial.println("RE-ARMED");
      }
      return;
    }

      //int32_t leftY = ctl->axisY();
      //targetBase = map(-leftY, -512, 512, 100, 1000);
      //base += (targetBase - base) * 0.05;
      unsigned long nowMs = millis();

      // Edge-detected: holding the pad would otherwise toggle every loop pass.
      bool dpadUp = ctl->dpad() & DPAD_UP;
      if (dpadUp && !lastDpadUp) {
        benchMode = !benchMode;
        Serial.printf("BENCH MODE %s - props off!\n", benchMode ? "ON" : "OFF");
      }
      lastDpadUp = dpadUp;

      if (nowMs - lastThrottleMs >= THROTTLE_REPEAT_MS) {
        if(ctl->a()) {  // Cross
          base = constrain(base + THROTTLE_STEP, IDLE_BASE, MAX_BASE);
          lastThrottleMs = nowMs;
        } else if(ctl->b()) {  // Circle
          base = constrain(base - THROTTLE_STEP, IDLE_BASE, MAX_BASE);
          lastThrottleMs = nowMs;
        }
      }
     
      
      unsigned long now = micros();
      dt = (now - lastTime) / 1000000.0;
      lastTime = now;
      // A stalled/dropped controller callback can leave a huge gap; capping dt
      // stops one late frame from dumping a giant step into the integrator.
      dt = constrain(dt, 0.0, 0.05);

      // Soft start only: ramp correction strength in just above idle so the
      // motors do not snap to a full correction the instant throttle is cracked.
      // The mixer enforces the real physical limit by scaling corrections into
      // the headroom around the current throttle.
      double authority = constrain((base - IDLE_BASE) / (FULL_AUTHORITY_BASE - IDLE_BASE),
                                   MIN_AUTHORITY, 1.0);
      if (benchMode) { authority = 1.0; }

      // One I2C read per pass, so accel and gyro come from the same instant.
      mpu6050.read();
      sensors_vec_t acceleration = mpu6050.lastAcceleration();
      sensors_vec_t gyro = mpu6050.lastGyro();

      std::pair<double, double> pair = filter.nextAngle(gyro, acceleration, dt);
      double roll = pair.first;
      double pitch = pair.second;

      if (base <= GROUND_BASE) {
        // Sitting on the bench the quad cannot level itself, so the integrators
        // would wind to their limit and dump that offset into the motors the
        // moment throttle came up. P and D still run, so tilting the frame gives
        // an immediate, visible response.
        rollPid.resetIntegral();
        pitchPid.resetIntegral();
        yawPid.resetIntegral();
      }

      // Feed the gyro straight in as the derivative term. Differentiating the
      // filtered angle instead amplifies sensor noise and inherits the
      // complementary filter's lag, both of which weaken damping - and weak
      // damping is what shows up as oscillation.
      double rollResult  = rollPid.compute(0, roll, gyro.x, dt) * authority * ROLL_SIGN;
      double pitchResult = pitchPid.compute(0, pitch, gyro.y, dt) * authority * PITCH_SIGN;
      double yawResult   = yawPid.compute(0, gyro.z, dt) * authority;

      Motors motors = mixer.compute(base, rollResult, pitchResult, yawResult);
      esc.sendDShotPacket(motors.m1, motors.m2, motors.m3, motors.m4);

      if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = nowMs;
        printTelemetry(authority, roll, pitch, gyro, acceleration,
                       rollResult, pitchResult, yawResult, motors);
      }
  });
}


