#include <Arduino.h> 
#include "controller.h"
#include "esc.h"


uint16_t throttle = 100;
void setup() {
    Serial.begin(115200);
    
    
    Serial.println("Starting initialization...");
    
    //controllerSetup();
    escSetup();
    Serial.println("Initialization complete!");
}

void loop() {
   
    // bool dataUpdated = BP32.update();
    // if (dataUpdated && ps4Controller) {
    //   int32_t raw = ps4Controller->axisY();

    //   if (raw < -512) raw = -512;
    //   if (raw > 512) raw = 512;
    //   throttle = map(raw, -512, 512, 0, 2000);
    //   throttle = constrain(throttle + 48, 48, 2047);
    sendDshotReliable(100);
    delay(100);
    // }
}
