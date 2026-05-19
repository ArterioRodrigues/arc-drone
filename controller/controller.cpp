#include "controller.h"

Drone::Controller::Controller() {}
void Drone::Controller::setup() {
    Serial.println("Initializing Bluepad32...");
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *address = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", address[0], address[1], address[2], address[3], address[4],
                  address[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
}
void onConnectedController(ControllerPtr controller) { _controller = controller; }

void onDisconnectedController(ControllerPtr controller) { _controller = nullptr; }

ControllerPtr Drone::Controller::getController() { return _controller; }

void Drone::Controller::processController(Callback callback) {
    BP32.update();
    callback();
    
}
