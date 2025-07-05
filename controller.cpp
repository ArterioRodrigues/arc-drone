#include "controller.h"

ControllerPtr ps4Controller = nullptr;

void onConnectedController(ControllerPtr ctl) {
  ps4Controller = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  ps4Controller = nullptr;
}

void dumpController() {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ps4Controller->index(), ps4Controller->dpad(), ps4Controller->buttons(),
    ps4Controller->axisX(), ps4Controller->axisY(), ps4Controller->axisRX(), ps4Controller->axisRY(),
    ps4Controller->brake(), ps4Controller->throttle(), ps4Controller->miscButtons(),
    ps4Controller->gyroX(), ps4Controller->gyroY(), ps4Controller->gyroZ(),
    ps4Controller->accelX(), ps4Controller->accelY(), ps4Controller->accelZ()
  );
}

void processController() {
  bool dataUpdated = BP32.update();
  
  if (dataUpdated && ps4Controller && ps4Controller->isConnected() && ps4Controller->hasData()) {
    if (ps4Controller->isGamepad()) {
      Serial.println(ps4Controller->axisX());
      Serial.println(ps4Controller->axisY());
    }
  }
}

void controllerSetup() {
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}
