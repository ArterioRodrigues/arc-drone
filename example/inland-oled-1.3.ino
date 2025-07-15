// #include "inlandOLED/inlandOLED.h"


// InlandOLED oled(18, 23, 5, 2, 4);

// void draw(void) {
//     u8g2.clearBuffer();
//     int xOffset = 2;  
    
//     u8g2.setFont(u8g2_font_7x13_tf);
//     u8g2.drawStr(xOffset, 13, "ESP32 Display");
//     u8g2.drawLine(xOffset, 16, 128 - xOffset, 16);
    
//     u8g2.setFont(u8g2_font_6x10_tf);
//     u8g2.drawStr(xOffset, 30, "Hello World!"); 
//     u8g2.drawStr(xOffset, 43, "Hello Inland!");
//     u8g2.drawStr(xOffset, 56, "U8g2 Library");
    
//     u8g2.sendBuffer();
// }

// void setup(void) {
//     Serial.begin(115200);
//     oled.setup();
//     oled.flipScreen(true);
// }

// void loop(void) {
//     draw();
//     delay(50);
// }
