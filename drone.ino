
#include "esc/esc.h"
#include "esc/esc.cpp"

ESC esc(DSHOT::DSHOT300);
void setup(void) {
  Serial.begin(115200);
  esc.setup();
}

void loop() {
  esc.sendDShotPacket(100);
}
