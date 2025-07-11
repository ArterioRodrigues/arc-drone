
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
InlandOLED oled(18, 23, 5, 2, 4);

void draw() {
    oled.clearBuffer();
    oled.setFont(FONT_SIZE_7);
    oled.drawStr(2, 13, "ESP32 Display");
    oled.drawLine(2, 16, 128 - 2, 16);
    
    oled.setFont(FONT_SIZE_6);
    oled.drawStr(2, 30, "Hello World!"); 
    oled.drawStr(2, 43, "Hello Inland!");
    oled.drawStr(2, 56, "U8g2 Library");

    oled.sendBuffer();
}

void setup(void) {
    Serial.begin(115200);
    mpu.setup();
    oled.setup();
    oled.flipScreen(true);

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
