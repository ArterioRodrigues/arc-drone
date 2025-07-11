#include <U8g2lib.h>

// Try SH1106 controller first (most common cause of offset issues)
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 2, /* reset=*/ 4);

// Alternative: If it's still SSD1306 but with offset, try this instead:
// U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 2, /* reset=*/ 4);

void draw(void) {
    // Clear the internal memory
    u8g2.clearBuffer();
    
    // Adjust text position to compensate for offset (shift right by 2-4 pixels)
    int xOffset = 2;  // Try values 0, 2, 4, or -2 to find the right position
    
    // Title with medium font
    u8g2.setFont(u8g2_font_7x13_tf);
    u8g2.drawStr(xOffset, 13, "ESP32 Display");
    
    // Draw a line separator
    u8g2.drawLine(xOffset, 16, 128 - xOffset, 16);
    
    // Content with smaller font
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(xOffset, 30, "Hello World!");
    u8g2.drawStr(xOffset, 43, "Hello Inland!");
    u8g2.drawStr(xOffset, 56, "U8g2 Library");
    
    // Transfer internal memory to the display
    u8g2.sendBuffer();
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("Initializing SPI OLED display...");
    
    // Initialize the display
    u8g2.begin();
    
    
    // Optional: flip screen if required
    // u8g2.setFlipMode(1);
    
    Serial.println("OLED display initialized successfully!");
}

void loop(void) {
    draw();
    delay(50);
}
