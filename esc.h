#pragma once
#include <Arduino.h> 
#include <driver/rmt.h>

void escSetup();
void printPacketBinary(uint16_t packet, uint16_t throttle);
bool sendDshotReliable(uint16_t throttle);
