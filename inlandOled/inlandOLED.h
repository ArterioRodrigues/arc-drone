#pragma once
#include <U8g2lib.h>

class InlandOLED {
    public:
        InlandOLED(int clockPin, int dataPin, int csPin, int dcPin, int resetPin);
        void setup();
        void flipScreen(bool flip);
        void setFont(int fontSize);
        void drawStr(int x, int y, const char* str);
        void drawLine(int x0, int y0, int x1, int y1);
        void clearBuffer();
        void sendBuffer();
        ~InlandOLED();
        U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* getDisplay();

    private:
        U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* _u8g2;

}