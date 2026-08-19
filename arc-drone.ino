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
double base = IDLE_BASE;
double targetBase = IDLE_BASE;
unsigned long lastTime = 0;

PID rollPid(200, 0.1, 10);
PID pitchPid(200, 0.1, 10);
PID yawPid(0, 0, 0);

Filter filter;
Mixer mixer;


void setup(void) {
  Serial.begin(115200);
  controller.setup();
  mpu6050.setup();
  esc.setup();
  lastTime = micros();
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

      //int32_t leftY = ctl->axisY();
      //targetBase = map(-leftY, -512, 512, 100, 1000);
      //base += (targetBase - base) * 0.05;
      if(ctl->a()) base = constrain(base + 5, IDLE_BASE, 1000);  // X button
      if(ctl->b()) base = constrain(base - 5, IDLE_BASE, 1000);
     
      
      unsigned long now = micros();
      dt = (now - lastTime) / 1000000.0;
      lastTime = now;
      // A stalled/dropped controller callback can leave a huge gap; capping dt
      // stops one late frame from dumping a giant step into the integrator.
      dt = constrain(dt, 0.0, 0.05);

      if (base <= IDLE_BASE) {
        rollPid.reset();
        pitchPid.reset();
        yawPid.reset();
      }

      sensors_vec_t acceleration = mpu6050.getAcceleration();
      sensors_vec_t gyro = mpu6050.getGyro();

      std::pair<double, double> pair = filter.nextAngle(gyro, acceleration, dt);
      double roll = pair.first;
      double pitch = pair.second;

      double rollResult  = rollPid.compute(0, roll, dt);
      double pitchResult = pitchPid.compute(0, pitch, dt);
      double yawResult   = yawPid.compute(0, gyro.z, dt);

      Motors motors = mixer.compute(base, rollResult, pitchResult, yawResult);
      esc.sendDShotPacket(motors.m1, motors.m2, motors.m3, motors.m4);
  });
}


