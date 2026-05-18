
#include "controller/controller.h"
#include "controller/controller.cpp"
#include "esc/esc.h"
#include "esc/esc.cpp"
#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.cpp"
#include "inlandOLED/inlandOLED.h"
#include "inlandOLED/inlandOLED.cpp"

//drone::Controller controller;
ESC esc(DSHOT::DSHOT300);
// MPU6050 mpu6050; //Default to GPIO 21 (SDA) and 22 (SCL)
// InlandOLED oled(27, 26, 32, 33, 25);
// void draw(sensors_vec_t acceleration, sensors_vec_t gyro, float temperature) {
//     oled.clearBuffer();
 
//     oled.setFont(FONT_SIZE_6);
//     oled.drawStr(25, 10, "SENSOR READINGS");
//     oled.drawLine(0, 14, 1024, 14);

//     oled.setFont(FONT_SIZE_4);
//     char buffer[64];
//     snprintf(buffer, sizeof(buffer), "Pos: X=%+5.2f Y=%+5.2f Z=%+5.2f", gyro.x, gyro.y, gyro.z);
//     oled.drawStr(0, 28, buffer);

//     snprintf(buffer, sizeof(buffer), "Acc: X=%+5.2f Y=%+5.2f Z=%+5.2f", acceleration.x, acceleration.y, acceleration.z);
//     oled.drawStr(0, 40, buffer);

//     snprintf(buffer, sizeof(buffer), "Temp: %.2f\xB0""C", temperature);
//     oled.drawStr(0, 52, buffer); 

  
//     oled.sendBuffer();
// }

void setup(void) {
  Serial.begin(115200);
  //controller.setup();
  
//   mpu6050.setup();

//   oled.setup();
//   oled.flipScreen(true);

  esc.setup();
}

void loop() {

    // controller.processController([]() {
    //     ControllerPtr ctl = controller.getController();
    //     if(ctl->isConnected() && ctl->hasData()) {
    //         esc.sendDShotPacket(100);
    //     }
    // });

    esc.sendDShotPacket(1000);
    // sensors_vec_t acceleration = mpu6050.getAcceleration();
    // sensors_vec_t gyro = mpu6050.getGyro();
    // float temperature = mpu6050.getTemperature();

    // draw(acceleration, gyro, temperature);
}
