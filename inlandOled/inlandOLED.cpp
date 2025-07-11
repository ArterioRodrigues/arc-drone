#include "inlandOLED.h"

InlandOLED::InlandOLED(int clockPin, int dataPin, int csPin, int dcPin, int resetPin) {
    this->_u8g2 = new U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI(U8G2_R0, clockPin, dataPin, csPin, dcPin, resetPin);
}
void InlandOLED::setup() {
    Serial.println("Initializing SPI OLED display...");
    this->_u8g2->begin();
    Serial.println("OLED display initialized successfully!");
}
void InlandOLED::flipScreen(bool flip) {
    this->_u8g2->setFlipMode(flip);
}
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* InlandOLED::getDisplay() {
    return this->_u8g2;
}

