#include "controller/controller.h"
#include "controller/controller.cpp"
#include "esc/esc.h"
#include "esc/esc.cpp"
#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.cpp"
#include "inlandOLED/inlandOLED.h"
#include "inlandOLED/inlandOLED.cpp"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include "pid/filter.h"
#include "pid/filter.cpp"
#include "pid/mixer.h"
#include "pid/mixer.cpp"

MPU6050 mpu6050; //Default to GPIO 21 (SDA) and 22 (SCL)
ESC esc(DSHOT::DSHOT300);
//Drone::Controller controller;
                 
double dt = 0;
double base = 500;
unsigned long lastTime = 0;

PID rollPid(0, 0, 0);
PID pitchPid(0, 0, 0);
PID yawPid(0, 0, 0);

Filter filter;
Mixer mixer;

void setup(void) {
  Serial.begin(115200);

  //controller.setup();
  mpu6050.setup();
  esc.setup();
}

void loop() {

    // controller.processController([]() {
    //     ControllerPtr ctl = controller.getController();
    //     if(ctl->isConnected() && ctl->hasData()) {
    //         esc.sendDShotPacket(100);
    //     }
    // });
    

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  sensors_vec_t acceleration = mpu6050.getAcceleration();
  sensors_vec_t gyro = mpu6050.getGyro();
     
  std::pair<double, double> pair = filter.nextAngle(gyro, acceleration, dt);
  double roll = pair.first;
  double pitch = pair.second;

  double rollResult = rollPid.compute(0, roll, dt);
  double pitchResult = pitchPid.compute(0, pitch, dt);
  double yawResult = yawPid.compute(0, gyro.z, dt);

  Motors motors = mixer.compute(base, rollResult, pitchResult, yawResult);
  
  esc.sendDShotPacket(motors.m1, motors.m2, motors.m3, motors.m4);
}
