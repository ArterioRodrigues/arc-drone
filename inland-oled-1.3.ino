#include <U8g2lib.h>

// SPI OLED Connection (based on your pins: 18, 23, 5, 2, 4)
// Your pins: U8GLIB_SSD1306_128X64 u8g(18, 23, 5, 2, 4);
// Pin mapping: (CLK/SCK, MOSI/SDA, CS, DC/A0, RESET)
// ESP32 SPI pins: 18=SCK, 23=MOSI, 5=CS, 2=DC, 4=RESET

U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 2, /* reset=*/ 4);

void draw(void) {
    // Clear the internal memory
    u8g2.clearBuffer();
    
    // Set font
    u8g2.setFont(u8g2_font_unifont_tf);
    
    // Draw text
    u8g2.drawStr(0, 22, "Hello World!");
    u8g2.drawStr(0, 43, "Hello Inland!");
    
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
