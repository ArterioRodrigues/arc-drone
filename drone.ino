
#include "controller/controller.h"
#include "controller/controller.cpp"
#include "esc/esc.h"
#include "esc/esc.cpp"
#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.cpp"

Controller controller();
ESC esc(DSHOT::DSHOT300);
MPU6050 mpu6050(); //Default to GPIO 21 (SDA) and 22 (SCL)

void draw() {

}

void setup(void) {
  Serial.begin(115200);
  controller.setup();
  esc.setup();
  mpu6050.setup();
}

void loop() {

    controller.processController([]() {
        ControllerPtr ctl = controller.getController();
        if(ctl.isConnected()) {
            esc.sendDShotPacket(100);
        }
    })


    sensors_vec_t acceleration = mpu6050.getAcceleration();
    sensors_vec_t gyro = mpu6050.getGyro();
    float temperature = mpu6050.getTemperature();

    draw(acceleration, gyro, temperature);
}
