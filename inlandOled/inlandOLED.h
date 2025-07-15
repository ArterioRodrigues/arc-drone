#pragma once
#include <U8g2lib.h>

enum FontSize {
    FONT_SIZE_4 = 4,
    FONT_SIZE_5 = 5,
    FONT_SIZE_6 = 6,
    FONT_SIZE_7 = 7,
    FONT_SIZE_8 = 8,
    FONT_SIZE_9 = 9,
    FONT_SIZE_10 = 10,
    FONT_SIZE_11 = 11,
    FONT_SIZE_12 = 12
};
// InlandOLED oled(18, 19, 5, 16, 17); // Example pin numbers for clock, data, CS, DC, reset
class InlandOLED {
    public:
        InlandOLED(int clockPin, int dataPin, int csPin, int dcPin, int resetPin);
        void setup();
        void flipScreen(bool flip);
        void setFont(FontSize fontSize);
        void drawStr(int x, int y, const char* str);
        void drawLine(int x0, int y0, int x1, int y1);
        void clearBuffer();
        void sendBuffer();
        
        uint8_t getStrWidth(const char* str);
        uint8_t getDisplayWidth();
        ~InlandOLED();
        U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* getDisplay();

    private:
        U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* _u8g2;

};
