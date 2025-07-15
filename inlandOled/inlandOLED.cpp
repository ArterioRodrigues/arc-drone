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
void InlandOLED::setFont(FontSize fontSize) {
    switch (fontSize) {
        case 4:
            this->_u8g2->setFont(u8g2_font_4x6_tf);
            break;
        case 5:
            this->_u8g2->setFont(u8g2_font_5x8_tf);
            break;
        case 6:
            this->_u8g2->setFont(u8g2_font_6x10_tf);
            break;
        case 7:
            this->_u8g2->setFont(u8g2_font_7x13_tf);
            break;
        case 8:
            this->_u8g2->setFont(u8g2_font_8x13_tf);
            break;
        case 9:
            this->_u8g2->setFont(u8g2_font_9x18_tf);
            break;
        case 10:
            this->_u8g2->setFont(u8g2_font_10x20_tf);
            break;
        case 11:
            this->_u8g2->setFont(u8g2_font_helvB08_tf);
            break;
        case 12:
            this->_u8g2->setFont(u8g2_font_ncenB10_tf);
            break;
        default:
            Serial.print("Unsupported font size: ");
            Serial.print(fontSize);
            Serial.println(". Using default 6x10.");
            this->_u8g2->setFont(u8g2_font_6x10_tf);
    }
}

void InlandOLED::drawStr(int x, int y, const char* str) {
    this->_u8g2->drawStr(x, y, str);
}
void InlandOLED::drawLine(int x0, int y0, int x1, int y1) {
    this->_u8g2->drawLine(x0, y0, x1, y1);
}
void InlandOLED::clearBuffer() {
    this->_u8g2->clearBuffer();
}
void InlandOLED::sendBuffer() {
    this->_u8g2->sendBuffer();
}
InlandOLED::~InlandOLED() {
    delete this->_u8g2;
}
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI* InlandOLED::getDisplay() {
    return this->_u8g2;
}

uint8_t InlandOLED::getDisplayWidth() {
    return this->_u8g2->getDisplayWidth();
}

 uint8_t InlandOLED::getStrWidth(const char* str){
    return this->_u8g2->getStrWidth(str);
 }
