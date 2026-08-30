# How the arc-drone stabilisation works

Reference for the flight code in `pid/` and `arc-drone.ino`. Angles are in
radians, motor commands in DShot units (48–2047).

## Signal chain

```
MPU6050 ──┬── gyro ─────────────────────────┐
          │                                 │ (D term, raw rate)
          └─▶ Filter::nextAngle()           │
                 complementary filter       │
                      │ roll/pitch (rad)    │
                      ▼                     │
                 PID::compute() ◀───────────┘
                      │ correction, DShot units
                      ▼
             Mixer::compute(base, roll, pitch, yaw)
                      │ four throttles
                      ▼
             ESC::sendDShotPacket()
```

| File                  | Responsibility                                    |
| --------------------- | ------------------------------------------------- |
| `mpu6050/mpu6050.cpp` | Raw sensor reads, gyro bias subtraction           |
| `pid/filter.cpp`      | Fuse accel + gyro into an attitude estimate       |
| `pid/pid.cpp`         | Attitude error → correction magnitude             |
| `pid/mixer.cpp`       | Corrections → four motor commands; owns all signs |
| `esc/esc.cpp`         | DShot frames over the ESP32 RMT peripheral        |

The PID knows nothing about motors; the mixer knows nothing about control
theory. The PID emits one scalar per axis and the mixer decides which motors it
helps and which it hurts.

## The control loop

`loop()` spins far faster than the control rate, so a rate gate defines it:
`CONTROL_PERIOD_US = 4000` → 250 Hz, matched to the DShot300 frame interval.

`dt` scales both the gyro integration (`gyro.x * dt`) and the integral term
(`Ki * error * dt`), so a wrong `dt` breaks both by the same factor. It must be
real elapsed time, measured once per control iteration. `constrain(dt, 0, 0.05)`
stops one late frame — a controller dropout — from dumping a step into the
integrator.

One I²C read per pass, so accel and gyro come from the same instant. Reading
them separately fuses two different moments and shows up as phase error the
filter cannot distinguish from real motion.

## Complementary filter (`pid/filter.cpp`)

The accelerometer measures gravity, so it knows absolute vertical and never
drifts — but it also measures every other acceleration, so thrust, translation
and vibration read as fake tilt. The gyro measures rate: smooth, fast, immune to
linear acceleration, but integrating it accumulates bias forever.

```c
double alpha = _timeConstant / (_timeConstant + dt);
_roll = alpha * (_roll + gyro.x * dt) + (1 - alpha) * accelAngles.first;
```

"Advance my previous estimate by the gyro, then nudge it toward the
accelerometer."

**Why alpha comes from `dt`:** a hard-coded `alpha = 0.98` has a real time
constant of `alpha * dt / (1 - alpha)`, which changes silently with loop rate.
Deriving it from `dt` pins the behaviour to a time constant in seconds.

With `FILTER_TIME_CONSTANT = 3.0` and `dt = 0.004`, `alpha = 0.9987` — each pass
is 99.87% gyro. But at 250 passes/sec the accelerometer is still the only thing
setting the DC level. Crossover is `1 / (2π × 3.0) ≈ 0.05 Hz`: above it the gyro
defines the estimate, below it the accelerometer does, completely.

> **Any steady error in the accelerometer becomes, within about a time
> constant, a steady error in the angle the controller is trying to zero.**

A long time constant _delays_ accel corruption; it does not reject it. A
transient survives, a sustained offset wins outright.

**Why 3.0 and not 1.0.** The accelerometer measures specific force, not gravity.
A quad that is leaning *and accelerating in the direction of that lean* has its
thrust along body z, so the horizontal components that would reveal the tilt
largely cancel: the accelerometer reports level while the craft is demonstrably
not. Magnitude stays near 1 g, so the trust gate below does not fire either -
the reading is wrong but entirely plausible.

This was observed directly. The craft flew away in a consistent direction with a
visible lean while the flight recorder logged an airborne pitch of 0.02 deg. At a
1.0 s constant the false "level" won within about a second and a genuine lean
dissolved out of the estimate, so the controller held the tilt believing it was
flat. Pilot trim could not fix it either: trim offsets the estimate, but the
accelerometer kept dragging the estimate back, and the absolute reference wins.

3.0 s rides out the transient. The cost is gyro bias, which is calibrated at
every boot and leaves roughly 0.3 deg of drift over a 3-second window.

**Accel trust gate.** Samples whose magnitude is outside 0.8–1.2 g are not
measuring gravity alone, so `alpha` is forced to 1.0 and the estimate coasts on
the gyro. This catches transients — spikes, strikes, hard manoeuvres. It does
**not** catch sustained vibration that rectifies into a DC offset while keeping
the magnitude near 1 g; soft-mounting the IMU is the only fix for that.

**Angles from the accelerometer.** `anglesFromAccel()` is where a static tilt is
read out of gravity:

```c
return {atan2(accel.y, accel.z),
        atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z))};
```

Roll is exact at any attitude — both components carry the same `cos(pitch)`
factor and the ratio cancels it. Pitch has no such luxury: the obvious mirror,
`atan2(-x, z)`, is correct only while roll is zero, because `z` shrinks with
roll and the inflated ratio reports a pitch that is not there.

| true roll / pitch | `atan2(-x, z)` | correct |
| ----------------- | -------------- | ------- |
| 30° / 30°         | 33.7°          | 30°     |
| 45° / 45°         | 54.7°          | 45°     |
| 60° / 20°         | 36.1°          | 20°     |

The error appears only when **both** axes are tilted, so pure-roll and
pure-pitch bench tests both pass and the fault surfaces only in combined motion
— as a phantom pitch error the controller then dutifully corrects for, and which
the integrator happily accumulates.

**Trim.** `calibrateLevel()` averages 500 resting accel samples at boot and
stores the implied angles, which become the definition of level for the whole
stack. Without it, an IMU mounted 2° off makes the controller hold a 2° bank
forever — flying exactly as commanded, just commanded wrongly, accelerating
sideways at `9.81 × sin(2°) ≈ 0.34 m/s²` with nothing to pull it back. It is
captured once, motors off, so anything that shifts the accel DC level afterwards
leaves it stale.

## The PID (`pid/pid.cpp`)

```c
if (!(dt > 0.0)) { return _lastOutput; }
double error = setpoint - measure;
_Iterm = clamp(_Iterm + _Ki * error * dt, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
_lastOutput = _Kp * error + _Iterm - _Kd * measureRate;
```

Setpoint is hard-coded 0 — self-levelling, no pilot attitude input. The only
pilot authority is collective throttle.

The guard is written `!(dt > 0.0)` rather than `dt <= 0.0` so it also rejects
NaN. A NaN `dt` would give a permanently stuck NaN `_Iterm` (NaN propagates
through the clamp and every later addition), silently killing the axis.

**P** is the spring. Gains are _per radian_, which makes them look large:

| Roll error | error (rad) | P output (Kp = 200) |
| ---------- | ----------- | ------------------- |
| 1°         | 0.0175      | 3.5                 |
| 10°        | 0.1745      | 34.9                |
| 45°        | 0.785       | 157                 |

Against a hover base of ~450, a 10° lean buys ~35 units — about 1.7% of the
throttle range. P alone always leaves steady-state error: at zero error it
produces zero output, so it cannot hold against a constant disturbance.

**I** exists to kill that offset. It is added straight to the output, _not_
multiplied by Kp, so it is independent of Kp — the same `Ki` gives roll and
pitch equal trim authority despite different Kp. At `Ki = 50` it builds at
`50 × error` units/sec, reaching the ~10 units that cancel a 3° bank in about
4 seconds.

> **`Ki` is currently 0 on both axes.** It is not needed until the craft holds a
> hover, and it is actively dangerous before then — see "Integrator saturation"
> below. Restore it to 50 only after hover throttle is measured and
> `LIFTOFF_BASE` is set above it.

Note the asymmetry in how I recovers. It winds up at `Ki × error`, which is fast
when the error is large — but it also _unwinds_ at `Ki × error`, so once it is
saturated and the craft is sitting level, the error driving the unwind is
nearly zero and it barely moves. At 0.002 rad it sheds 0.12 units/sec: over five
minutes to walk back from a limit of 40. A saturated integrator is, in practice,
a permanent offset.

Its two failure modes:

1. **Windup** — if the craft physically cannot reduce the error (grounded, or
   saturated at the motor limits), I accumulates against a wall and releases it
   as a lurch. `PID_INTEGRAL_LIMIT` and the liftoff latch both fight this.
2. **Chasing a false zero** — if the _measurement_ is wrong, P settles at a
   partial error and stops, but I refuses to stop until the wrong reading says
   zero, driving the craft into a real bank to satisfy a fake one. I is the term
   that converts a sensor error into a crash.

`PID_INTEGRAL_LIMIT` is 40, roughly P's output at a 10° lean. Sizing it against
P matters: at the old value of 200 the integrator could reach the equivalent of
a 57° P error and outvote P about 6:1 — an authority limit dressed up as a
windup guard.

**D** is the damper, resisting rate of change so it opposes the overshoot P
creates. Without it, P plus airframe inertia is a mass on a spring with no
friction. **Never test with `Kd = 0`.** Two choices here:

- _Derivative on measurement, not error._ Textbook `Kd * d(error)/dt` gives an
  infinite derivative on a setpoint step ("derivative kick"). `-Kd * d(measure)/dt`
  behaves identically for disturbance rejection with no kick; the sign flips
  because differentiating `-measure` negates it.
- _Gyro fed in directly_ rather than differentiating the filtered angle, which
  would amplify noise (differentiation is a high-pass) and inherit the filter's
  lag. Both weaken damping, and weak damping shows up as oscillation. The gyro
  _is_ the derivative, measured with no lag — it's free.

**Yaw is a rate controller.** It is fed `gyro.z`, a rate, not an angle, so its
`Kp` is per rad/s and the roll/pitch numbers are not comparable — 200 there
would be colossal, not equivalent; ~30 is the right order. `Kd` stays 0 because
there is no yaw acceleration signal. All three gains are currently 0 because
hover testing showed the craft holds heading unaided. Run bench step 5 before
giving yaw any gain: an inverted yaw axis is positive feedback that spins up.

## The mixer (`pid/mixer.cpp`)

Physical layout as flown, viewed from above (nose up the page):

```
   m1 (CW)  ────  m2 (CCW)
      │   \    /   │
      │     ><     │
      │   /    \   │
   m3 (CCW) ────  m4 (CW)
```

* `m1`/`m4` are the clockwise diagonal, `m2`/`m3` the counter-clockwise one.
* Each prop must match its motor — a CW motor needs a CW prop.
* Bench-verified: a prop's reaction torque on the frame is opposite to its own
  rotation, so a clockwise yaw is countered by spooling up the CW pair `m1`/`m4`
  (their reaction torque is CCW). That is why yaw is added to `m1`/`m4` and
  subtracted from `m2`/`m3` below. Spooling up `m2`/`m3` instead is positive
  feedback: it flies fine on the bench and spins up the instant it leaves the
  ground.

```c
double correction[4] = {
    +roll - pitch + yaw,  // m1 front-left
    -roll - pitch - yaw,  // m2 front-right
    +roll + pitch - yaw,  // m3 back-left
    -roll + pitch + yaw,  // m4 back-right
};
```

Roll splits left (m1/m3) against right (m2/m4); pitch splits front (m1/m2)
against back (m3/m4); yaw splits the diagonals by prop rotation, because yaw
torque comes from the reaction to prop drag rather than from thrust.

The PID output is **already negated** (`error = 0 - angle`). Worked example for
positive roll, right side down: `rollResult` is negative → m2 gets `-roll`,
minus a negative → m2 speeds up → the right (low) side lifts → level.

**All sign decisions live in this one file.** The alternative — a `ROLL_SIGN`
multiplier upstream — is arithmetically equivalent but gives you two places to
look that can disagree, and a single upstream negation flips P and D together
when only one of them is inverted. Fix a wrong-way axis here or in the wiring;
the bench test, not a re-derivation, is the authority on which.

**Headroom scaling.** The naive `constrain(base + correction[i], 48, 2047)` per
motor is a trap: attitude is controlled by the _differences_ between motors, so
two motors both clamped at 48 become identical and roll authority collapses
precisely when the craft is most out of shape. Scaling the whole spread
preserves every ratio, so you lose magnitude but never direction:

```c
double scale = 1.0;
if (highest > 0.0) { scale = min(scale, (MOTOR_MAX - base) / highest); }
if (lowest  < 0.0) { scale = min(scale, (base - MOTOR_MIN) / -lowest); }
```

This scales rather than _shifting_ the base throttle to make room. A shift is
what many flight controllers do, but it spools every motor up without the pilot
asking. At a hover base of 450 the headroom is 1597 up and 402 down, so `scale`
is 1.0 for any realistic correction — this only bites near the extremes.

## Safety machinery

**Kill switch (Triangle).** Latched, and deliberately survives a controller
dropout — once killed nothing spins until the two-button re-arm (Square + L1),
so a stray press cannot restart the props. It writes zeros straight to the ESCs,
bypassing `base` and the mixer so no PID output can leak through, and resets all
three PIDs. It still sends a real DShot frame so the ESCs stay armed.

**Keep-alive.** With no controller connected, `esc.keepAlive()` keeps the ESCs
from timing out, and `lastTime` is refreshed so the first frame after
reconnection does not produce a multi-second `dt`.

**Liftoff latch.** Throttle is the only airborne proxy available — no altimeter,
no weight-on-wheels switch. While grounded the craft cannot correct its attitude
no matter what the motors do, so every grounded pass winds the integrator
against an error it is powerless to fix, and that offset dumps into the motors
as a bank the moment it goes light.

`LIFTOFF_BASE` must sit **above** hover: passing it means the craft is climbing,
which it cannot do while grounded. Because a hovering craft then sits back below
the threshold, the result is **latched** rather than tested each pass —
otherwise the integrator would switch off in the one regime it exists to serve.
The latch clears only on kill, the one moment the craft is known to be back on
the ground. P and D still run below the latch, so bench response to tilting
stays live.

**Hover throttle is not yet measured.** The threshold was 500, derived from an
assumed ~450 hover; telemetry later showed the craft still firmly on the ground
at base 630, so the gate was opening while grounded on every run — the exact
failure it exists to prevent. It is now parked at 1200, deliberately high:
erring high only means the latch never trips, which costs nothing at `Ki = 0`,
while erring low re-creates the tip-over. Measure hover, then set it just above.

**Telemetry is compile-time gated** (`TELEMETRY_ENABLED 0`) rather than
commented out, so the body still has to compile and cannot rot. The line is
~185 characters, ~16 ms at 115200 baud, and `Serial` blocks once its TX buffer
fills — that stall lands inside the control loop.

## Bench procedure — props off

Do these in order. Steps 1/2 cover roll, 3/4 pitch, 5 yaw. A passing roll check
says nothing about pitch: the left pair is m1/m3 and the right pair m2/m4
whichever end is the front, so a front/back swap sails through steps 1–2 and
only reveals itself in the air.

1. **Roll axis agreement.** Rock right and left. Rolling right, `roll` and
   `accRoll` must both increase _and_ `gyroX` must be positive. If gyroX opposes
   them, gyro and accel disagree on sign: the filter fights itself and D becomes
   anti-damping, which no gain tuning can fix. Fix the axis mapping.
2. **Roll motor direction.** At idle, tilt right side down — m2/m4 must speed
   up. If m1/m3 do, swap the left and right motor pairs in the wiring.
3. **Pitch axis agreement.** Nose down: `pitch` and `accPitch` increase together
   while `gyroY` is positive. Same failure mode as step 1.
4. **Pitch motor direction.** Nose down, the physical **back** pair must speed
   up (the mixer's back pair is m3/m4). If the front motors speed up, swap the
   m1/m2 leads with m3/m4.
5. **Yaw motor direction.** Only needed once yaw gains are non-zero. There is no
   step-1 equivalent — gravity gives an absolute tilt reference, nothing gives
   one for heading. Set yaw to (30, 0, 0) temporarily, note each motor's spin
   direction, then rotate the frame nose-right: the **clockwise-spinning** pair
   (m1/m4) must speed up, because their reaction torque on the frame is CCW. If
   the counter-clockwise pair does, yaw is backwards — fix the prop/motor
   rotation directions, not the software.
6. Only once all of the above pass, fit props.

Never fix a wrong-way axis by negating something upstream; see the mixer note.

## Tuning order

1. Props off, verify direction on every axis (bench steps 1–5). Direction before
   magnitude, always.
2. **P only** (`Ki = 0`, small `Kd`). Raise Kp until it oscillates when
   disturbed, then back off ~30%.
3. **Add D** until the oscillation damps. Too much D amplifies noise and the
   motors hiss.
4. **Add I last**, once the craft holds a clean hover — you cannot see a steady
   lean through an unstable one. Keep it small; if I ever needs to be large,
   something upstream is wrong and raising it only hides that.

Change one gain at a time.

## Debugging a craft that leans and never recovers

### Integrator saturation — check this first

The cheapest check, and the one that has already caught a real fault. Sit the
craft level, throttle up with props off, and read one telemetry line:

```
roll=-0.002 accRoll=-0.009 gyroX=0.000 | pitch=-0.002 accPitch=-0.071 gyroY=0.001
| pid r= -33.4 p= +40.2 | m1=556 m2=623 m3=637 m4=704 | dt=0.0040   (base=630)
```

The craft is level to within 0.1°, so P can only be contributing
`200 × 0.002 = 0.4` units — yet the outputs are ±33 and ±40, and `p = 40.2` is
pinned exactly at `PID_INTEGRAL_LIMIT`. **Every one of those units is
integrator.** The 148-unit spread between `m1` and `m4` is a hard diagonal bank
commanded while the craft is sitting flat.

That is the tell: **large `pid` outputs while `roll` and `pitch` read near
zero.** The loop is not failing to correct a tilt, it is producing one. On the
floor it tips the craft over, always the same way, and never recovers — and
because the unwind rate is `Ki × error`, it will not clear on its own inside
five minutes.

The cause is upstream: the integrator was allowed to run while grounded, where
the craft is powerless to reduce the error, so it wound to its limit against a
wall. Fix the gate (`LIFTOFF_BASE`), not the limit — raising
`PID_INTEGRAL_LIMIT` only buys a bigger bank.

### Everything else

The important distinction: the controller may not be failing to correct — it may
be **correcting perfectly to a zero that isn't level.** Those look identical
from outside. If the axis responds correctly when tilted by hand, sign inversion
is ruled out, leaving, in rough order of likelihood:

1. **Vibration corrupting the accel DC level.** The accelerometer owns the DC
   attitude completely, and MPU6050 accel noise rectifies into an offset that
   grows with RPM. Fits "progressive with RPM, one axis, never recovers"
   exactly. The `MPU6050_BAND_44_HZ` DLPF helps but does not stop aliasing from
   prop frequencies.
2. **The integrator chasing that false zero** — mostly an amplifier of #1 rather
   than an independent cause.
3. **Not enough authority for a real asymmetry** (thrust mismatch, CG offset,
   bent arm). Tilting by hand confirms _direction_, not _strength_.
4. **Ordinary tip-over.** `THROTTLE_STEP = 5` per 50 ms means ~3.5 s creeping
   through the band where the craft is light on its feet but not flying, and no
   controller can fix an attitude while a leg is in contact. Real quads leave
   the ground briskly for exactly this reason.

**The test, no code change, no props:** set `TELEMETRY_ENABLED 1`, clamp the
frame level, walk the throttle up and watch `accRoll`.

- Drifts from zero as RPM rises → #1. Soft-mount the IMU, lower the DLPF.
- Stays put but motor outputs diverge → #2/#3. Watch whether `pid r=` grows
  while `roll` stays near zero; that is the integrator winding.
- Everything clean → #4, mechanical/procedural; take off decisively.

Do this before changing gains. Tuning against a lying sensor produces gains that
are wrong once the sensor is fixed.
