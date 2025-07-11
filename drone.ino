#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "mpu6050/mpu6050.h"
#include "controller/controller.h"

Adafruit_MPU6050 mpu;
uint16_t mapInputToThrottle(int32_t input) {
    if (input < -512) input = -512;
    if (input > 512) input = 512;
    return (uint16_t)((input + 512) * MOTOR_RANGE / CONTROLLER_RANGE);
}

void setup(void) {
    Serial.begin(115200);
    MPU6050 mpu;
    Controller controller;
    DShot600 esc;
}

void loop() {
    mpu.dumpSensorDetails();
    controller.processController([]() {
        ControllerPtr controller = controller.getController();
        if (controller && controller->isConnected() && controller->hasData()) {
            int32_t axisX = controller->axisX();
            int32_t axisY = controller->axisY();
            esc.sendDshotPacket(mapInputToThrottle(axisY));
        }
    });

    delay(500);
}