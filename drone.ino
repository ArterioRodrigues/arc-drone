
#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.cpp"
// #include "controller/controller.h"
// #include "dshot600/dshot600.h"

// #define MOTOR_RANGE 2048
// #define CONTROLLER_RANGE 1024
// uint16_t mapInputToThrottle(int32_t input) {
//     if (input < -512) input = -512;
//     if (input > 512) input = 512;
//     return (uint16_t)((input + 512) * MOTOR_RANGE / CONTROLLER_RANGE);
// }
MPU6050 mpu;
void setup(void) {
    Serial.begin(115200);
    mpu.setup();
    // Controller controller;
    // DShot600 esc;
}

void loop() {
    mpu.dumpSensorDetails();
  
    // controller.processController([]() {
    //     ControllerPtr controller = controller.getController();
    //     if (controller && controller->isConnected() && controller->hasData()) {
    //         int32_t axisX = controller->axisX();
    //         int32_t axisY = controller->axisY();
    //         esc.sendDshotPacket(mapInputToThrottle(axisY));
    //     }
    // });

    delay(500);
}
