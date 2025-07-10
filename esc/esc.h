#pragma once
#include <Arduino.h>
#include <cstdint>
#include <driver/rmt.h>

class Esc {
    public:
        Esc();
        void printPackBinary(uint16_t packet, uint16_t throttle);
        bool sendDshotReliable(uint16_t throttle);
}
