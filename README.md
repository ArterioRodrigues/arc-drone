# arc-drone

A modular, ESP32-based flight stack for a custom quadcopter. `arc-drone` bundles a small set of Arduino/C++ drivers — ESC (DShot), IMU, OLED telemetry display, and a Bluetooth gamepad controller — behind clean interfaces so they can be combined inside a single sketch (`drone.ino`) or reused independently.

## Features

- **Digital ESC driver** with DShot150 / DShot300 / DShot600 support
- **Bluetooth gamepad input** (PS4/PS5/Xbox/Switch controllers via Bluepad32)
- **MPU6050 IMU driver** for accelerometer, gyroscope, and temperature readings over I²C
- **SPI OLED driver** (Inland 0.96" 128×64) for live on-board telemetry
- **Single entry-point sketch** (`drone.ino`) that wires everything together

## Hardware

| Component | Notes |
|---|---|
| MCU | ESP32 (uses ESP32-only RMT peripheral and Bluetooth stack) |
| ESC | Any 4-in-1 / single ESC supporting DShot150/300/600 |
| IMU | MPU6050 (I²C) |
| Display | Inland 0.96" SPI OLED (SSD1306-class) |
| Input | Bluetooth gamepad supported by [Bluepad32](https://github.com/ricardoquesada/bluepad32) |

### Default pin map

| Peripheral | Signal | GPIO |
|---|---|---|
| MPU6050 | SDA | 21 |
| MPU6050 | SCL | 22 |
| OLED | (5 pins, SPI-style) | 27, 26, 32, 33, 25 |
| ESC | DShot output | configured via the `ESC` constructor |

Pins are passed into the driver constructors, so you can remap them to fit your wiring.

## Repository layout

```
arc-drone/
├── controller/     # Bluetooth gamepad driver (Bluepad32 wrapper)
├── esc/            # DShot ESC driver
├── mpu6050/        # IMU driver (accel / gyro / temperature)
├── inlandOled/     # SPI OLED display driver
├── example/        # Standalone example sketches per driver
├── drone.ino       # Top-level sketch that ties the drivers together
└── .vscode/        # Editor settings
```

## Getting started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (or [arduino-cli](https://arduino.github.io/arduino-cli/))
- ESP32 board support package installed via the Arduino Boards Manager
- Required libraries:
  - **Bluepad32** — Bluetooth gamepad input
  - **Adafruit MPU6050** + **Adafruit Unified Sensor** — IMU
  - A DShot library for ESP32 (e.g. `DShotRMT`) — ESC output
  - **U8g2** — OLED rendering

> Exact library names and versions depend on what's `#include`d inside each driver's `.h` / `.cpp`. Open the corresponding folder for specifics.

### Build & flash

1. Clone the repo:
   ```bash
   git clone https://github.com/ArterioRodrigues/arc-drone.git
   ```
2. Open `drone.ino` in the Arduino IDE.
3. Select your ESP32 board and the correct serial port.
4. Click **Upload**.

## Usage

The top-level sketch (`drone.ino`) shows the intended pattern: each driver is constructed at file scope, initialized in `setup()`, and driven from `loop()`.

```cpp
#include "controller/controller.h"
#include "esc/esc.h"
#include "mpu6050/mpu6050.h"
#include "inlandOLED/inlandOLED.h"

drone::Controller controller;
ESC               esc(DSHOT::DSHOT300);
MPU6050           mpu6050;                          // default I²C pins (SDA=21, SCL=22)
InlandOLED        oled(27, 26, 32, 33, 25);

void setup() {
  Serial.begin(115200);
  controller.setup();
  esc.setup();
  mpu6050.setup();
  oled.setup();
}

void loop() {
  controller.processController([]() {
    ControllerPtr ctl = controller.getController();
    if (ctl->isConnected() && ctl->hasData()) {
      esc.sendDShotPacket(1000);                    // throttle value
    }
  });

  sensors_vec_t accel = mpu6050.getAcceleration();
  sensors_vec_t gyro  = mpu6050.getGyro();
  float         temp  = mpu6050.getTemperature();
  // ... render to OLED, run control loop, etc.
}
```

### Flight controls (PS4 layout)

| Input | Action |
|---|---|
| Cross (X) | Throttle up — `base` +1 per 50 ms while held (capped at 1000) |
| Circle | Throttle down — `base` −1 per 50 ms while held (floored at idle) |
| **Triangle** | **Kill switch** — latches: motors are commanded to zero and PID state is cleared |
| Square + L1 | Re-arm after a kill (two-button combo so a stray press can't restart the props) |
| D-pad Up | Toggle bench mode — forces full correction authority so the loop can be verified at low throttle. **Props off.** |

Throttle trim is rate-limited rather than applied every loop pass; adjust
`THROTTLE_STEP` and `THROTTLE_REPEAT_MS` in `arc-drone.ino` to taste.

### Startup calibration

`setup()` calibrates before the ESCs are armed, and **the craft must be
stationary and level** for both steps:

1. `mpu6050.calibrateGyro()` averages the gyro to measure its zero offset. The
   raw offset is a few degrees per second; integrated by the filter it walks the
   attitude estimate away from level.
2. `calibrateLevel()` averages the accelerometer to establish which way is down
   and records that attitude as level.

Step 2 matters more than it looks. An IMU mounted a couple of degrees off makes
the controller hold that tilt forever, and a 3° error at hover is about
`g·sin(3°) ≈ 0.5 m/s²` of sideways acceleration — the craft picks a direction and
accelerates away. It is flying exactly as commanded; the command is just wrong.

Both print their results over serial, so check the reported level reference is
near zero. Calibrating on a slope bakes that slope in.

### Attitude filter

The complementary filter blends integrated gyro with the accelerometer. The
blend is derived from `dt` and a fixed time constant:

```
alpha = FILTER_TIME_CONSTANT / (FILTER_TIME_CONSTANT + dt)
```

A hard-coded `alpha` instead gives a time constant of `alpha·dt/(1-alpha)`, which
changes silently with loop rate — at the rate this loop runs, `alpha = 0.98`
worked out to roughly 0.1 s.

That is too short, because the accelerometer only reads true gravity while the
craft is not accelerating. In flight it reads thrust plus gravity, so during any
translation it tilts and reports a lean the craft does not have. With a short
time constant that corrupts the estimate within a fraction of a second, the
controller "corrects" a tilt that is not there, and the craft accelerates away.

### Loop latency

Delay in a feedback path is what makes corrections arrive late, and late
corrections force gains down to stay stable. Sources that were removed:

| Source | Was | Now |
|---|---|---|
| MPU6050 on-chip low-pass | 5 Hz, ~19 ms group delay | 44 Hz, ~4.9 ms |
| I2C bus clock | 100 kHz, ~1.5 ms per sample | 400 kHz, ~0.4 ms |
| Telemetry `Serial.printf` | ~16 ms stall every 200 ms | compiled out |

That is roughly 15 ms of pure delay out of the loop.

Telemetry lives in `printTelemetry()` and is controlled by `TELEMETRY_ENABLED`
in `arc-drone.ino`. Set it to `1` for bench work and back to `0` before flying.
It is a compile-time switch rather than commented-out code so the body still has
to compile and cannot go stale. Event messages (kill, re-arm, bench mode) are
left on — they fire once, not every pass.

Raising the DLPF to 44 Hz lets more gyro noise through, which the D term
amplifies. If the motors get twitchy or hissy, drop `Kd` before reverting the
bandwidth; `MPU6050_BAND_21_HZ` (~8.5 ms) is the intermediate step.

### ESC keep-alive

`ESC::keepAlive()` resends the **last commanded** throttle when the flight loop
has not produced a frame within a frame period. It must not send neutral: the
flight path only runs when the controller reports new data, so a late or dropped
Bluetooth frame would cut the motors dead mid-air. Before anything is commanded
the stored throttle is zero, so ground behaviour is unchanged.

### Stabilization authority

Angles from the complementary filter are in **radians**, so the PID gains are
per-radian. `Kp = 200` is the value the airframe last flew with. It was briefly
raised to 300 while chasing a weak response, but that turned out to be a sign
problem rather than a gain problem, so it is back to the known-good number.
Tune from here, one change at a time.

The derivative term is fed the raw gyro rate rather than a numerical derivative
of the filtered angle. Differentiating the filtered angle amplifies sensor noise
and inherits the complementary filter's lag, and both weaken damping — which
shows up as oscillation.

Correction strength ramps in over `IDLE_BASE → FULL_AUTHORITY_BASE` and is full
above that:

```
authority = (base - IDLE_BASE) / (FULL_AUTHORITY_BASE - IDLE_BASE)   // clamped
```

This is only a soft start, so corrections are not jerky the instant throttle is
cracked off idle. The real physical limit is enforced by the mixer, which scales
corrections into whatever headroom the current throttle allows — so there is no
need to keep suppressing them all the way up to hover.

Authority never falls to zero; it is floored at `MIN_AUTHORITY` so the loop
always shows some response. Below `GROUND_BASE` only the integrator is held
clear, leaving P and D live so tilting the frame on the bench still moves the
motors.

`GROUND_BASE` must sit just **below actual hover throttle**, not down at idle.
The throttle trim climbs at 20 units/sec, so a threshold of 300 against a ~1400
hover left the integrator accumulating for ~55 seconds while the craft was still
on the ground — long enough to saturate at `iLimit` and dump a full-scale
correction into the motors the instant it got light. Retune it if hover throttle
changes.

### Yaw and prop directions

Yaw is a **rate** controller, not an angle controller. `compute()` is fed
`gyro.z` directly as the measurement, so `Kp` acts as rate damping — it resists
rotation rather than holding a heading. `Ki` stays `0`: there is no magnetometer,
so there is no absolute heading to hold and an integrator would wind up against a
reference that does not exist.

Yaw was previously unstabilised (`PID yawPid(0, 0, 0, ...)` — every gain zero, so
the yaw output was always exactly `0`). Nothing opposed rotation about the
vertical axis, while roll and pitch corrections actively induce it: motor torque
scales with speed, so an asymmetric mix twists the frame. The result is a craft
that yaws, keeps yawing, and spins in.

Yaw control only works if the props are arranged correctly. The mixer pairs them
diagonally — `m1`/`m4` take `+yaw`, `m2`/`m3` take `-yaw`:

```
   m1 (CW)  ────  m2 (CCW)
      │   \    /   │
      │     ><     │
      │   /    \   │
   m3 (CCW) ────  m4 (CW)
```

* `m1` and `m4` spin one direction; `m2` and `m3` spin the other.
* Each prop must match its motor — a CW motor needs a CW prop.

If a diagonal pair does not match, the frame has a permanent net torque and will
spin regardless of gains. **No amount of tuning fixes wrong prop directions**;
check this before touching `Kp`.

### Throttle and hover

`HOVER_BASE` is the throttle the airframe hovers at, and `GROUND_BASE` derives
from it at 85%. Below `GROUND_BASE` the integrator is held clear so it cannot
wind up against ground it has no authority to level off.

Set `HOVER_BASE` on the **low** side if unsure. Too high parks `GROUND_BASE`
above real hover, so the integrator never switches on and the drift it exists to
remove stays. Too low only widens the ground window, and a craft sitting on flat
ground that was calibrated on that same ground reports near-zero error, so it
integrates almost nothing. Do not take off from a slope.

`Ki` on roll and pitch is currently staged at `0`. It exists to trim out steady
drift, but drift is tolerable and instability is not, so it stays out of the loop
until the craft holds a clean hover. Raise it to `50` once hover is stable and
the only remaining complaint is a slow lean. Until then the loop cannot null a
steady disturbance — P always stops short, so it settles wherever it balances the
disturbance and holds a small bank. That lean is expected, not a fault.

**Measuring hover throttle without a serial cable:** the throttle trim is a known
ramp — `THROTTLE_STEP` per `THROTTLE_REPEAT_MS`, currently 20 units/sec from
`IDLE_BASE`. Hold **Cross** from idle and time it with a stopwatch:

```
HOVER_BASE ≈ IDLE_BASE + 20 × seconds_to_lift_off
```

So lifting off after 35 s of held throttle means roughly `100 + 700 = 800`.

### Why the bench test looks weak

Tilting the frame on the ground at idle will not level it, and that is expected:

* Authority is scaled down at low throttle — at `base = 150` it is `0.167`, so
  corrections run at a sixth strength.
* The integrator is force-cleared below `GROUND_BASE`.
* At idle the props make almost no thrust, so tens of units of differential
  cannot lift the frame against the ground regardless of gains.

Use bench mode (D-pad Up, props off) to force `authority = 1.0`, and judge the
*direction and proportionality* of the motor numbers rather than whether the
craft physically levels.

Note that `Kd` can never affect a **static** tilt. The D term is `-Kd * gyro`,
and a craft sitting still has zero gyro rate, so D is exactly zero for any `Kd`.
A steady offset that will not go away is an `I` problem, not a `D` problem.

### Motor layout and mixing

Viewed from above:

```
  m1 front-left      m2 front-right
  m3 back-left       m4 back-right
```

Roll splits left (m1/m3) from right (m2/m4), pitch splits front (m1/m2) from
back (m3/m4), and yaw splits the two diagonals.

To correct a tilt the **low side** needs more thrust. The complementary filter
reports roll positive for right-side-down and pitch positive for nose-down, and
the PID computes `error = 0 - angle`, so its output is already negative for a
positive tilt — which means the low side is the one with the correction
*subtracted*. Getting this backwards turns the loop into positive feedback: the
craft drives itself further into the tilt and oscillates instead of levelling.

Verify on the bench (props off, D-pad Up for bench mode) before every flight
after touching wiring: tilt right, and `m2`/`m4` must rise.

The correct signs depend on how the IMU is physically mounted, which cannot be
determined from the code. `ROLL_SIGN` and `PITCH_SIGN` in `arc-drone.ino` exist
so an axis can be flipped without reworking the mixer:

| Bench result | Change |
|---|---|
| Tilt right raises m2/m4, nose down raises m1/m2 | Correct — leave both at `+1` |
| Tilt right raises m1/m3 | `ROLL_SIGN = -1.0` |
| Nose down raises m3/m4 | `PITCH_SIGN = -1.0` |
| Both backwards | Set both to `-1.0` |

**This airframe currently needs `ROLL_SIGN = -1.0`.** The bench test showed a
left tilt speeding up `m2`/`m4` — the high side — so the loop was driving itself
further into the tilt. That is what made the craft pick a direction and
accelerate away rather than level off. The mixer's own derivation assumes a
standard IMU orientation and is wrong here; these constants are the authority,
so do not re-derive the mixer signs from first principles.

A wrong sign is positive feedback, so test one axis at a time with props off.

The mixer never clamps motors individually. Attitude is set by the *differences*
between motors, so clipping one at the 48 floor would flatten the differential
and the quad would stop responding precisely when it most needs to. Instead the
whole correction set is scaled down to fit the headroom above and below the
current throttle, preserving the ratios. Base throttle is never shifted up to
make room, since motors spooling up unasked is dangerous in the hand.

The kill latch survives a controller dropout: once triggered, nothing spins again
until the re-arm combo is pressed. While killed the sketch keeps sending
zero-throttle DShot frames so the ESCs stay armed and respond instantly on re-arm.

### Serial telemetry

At 115200 baud the sketch prints a line every 200 ms with the current base
throttle, filtered roll/pitch, raw gyro and accelerometer vectors, PID outputs,
the four commanded motor values, and the loop `dt`:

```
base= 118.0 | roll=  -1.24 pitch=   0.87 | gyro x=  -0.01 y=   0.02 z=   0.00 | accel x= -0.11 y=  0.980 z=  9.79 | pid r=  248.0 p= -174.0 y=    0.0 | m1= 192 m2=  48 m3=  48 m4= 240 | dt=0.0043
```

Kill and re-arm events are printed as they happen.

### Driver overview

#### ESC (`esc/`)
Sends DShot frames to a brushless ESC over the ESP32's RMT peripheral.
```cpp
ESC esc(DSHOT::DSHOT300);   // also: DSHOT150, DSHOT600
esc.setup();
esc.sendDShotPacket(1000);  // 0 = disarmed, 48–2047 = throttle range
```

#### Controller (`controller/`)
Thin wrapper around Bluepad32. Pair a gamepad to the ESP32 once, then poll it each loop.
```cpp
drone::Controller controller;
controller.setup();
controller.processController([]() {
  ControllerPtr ctl = controller.getController();
  if (ctl->isConnected() && ctl->hasData()) {
    // read sticks / buttons here
  }
});
```

#### MPU6050 (`mpu6050/`)
I²C IMU driver returning accelerometer / gyroscope vectors and chip temperature.
```cpp
MPU6050 mpu;            // defaults to SDA=21, SCL=22
mpu.setup();
auto accel = mpu.getAcceleration();
auto gyro  = mpu.getGyro();
float t    = mpu.getTemperature();
```

#### InlandOLED (`inlandOled/`)
Buffered drawing API for an SPI 128×64 OLED. Useful for flight-side telemetry.
```cpp
InlandOLED oled(27, 26, 32, 33, 25);
oled.setup();
oled.flipScreen(true);

oled.clearBuffer();
oled.setFont(FONT_SIZE_6);
oled.drawStr(25, 10, "SENSOR READINGS");
oled.drawLine(0, 14, 128, 14);
oled.sendBuffer();
```

## Examples

Standalone sketches for each driver live in `example/`. They're the fastest way to verify wiring before plugging everything into `drone.ino`.

## Status

⚠️ Active development — APIs and pin maps may shift. The current `drone.ino` runs the ESC at a fixed throttle for bench testing; the gamepad → ESC control path and IMU-based stabilization loop are commented out as they're integrated.

## Safety

Always bench-test ESCs and motors **with propellers removed**. Brushless motors at full throttle can cause serious injury. Verify throttle range and arming behavior before mounting props.

## License

No license file is currently included. Add one (MIT / Apache-2.0 / etc.) before others can reuse the code.
