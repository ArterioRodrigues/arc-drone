#include "controller.h"

Controller::Controller() {
    Serial.println("Initializing Bluepad32...");
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* address = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", address[0], address[1], address[2], address[3], address[4], address[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void Controller::onConnectedController(ControllerPtr ctl) {
    controller = ctl;
}

void Controller::onDisconnectedController(ControllerPtr ctl) {
    controller = nullptr;
}

ControllerPtr Controller::getController() {
    return controller;
}

void Controller::processController(Callback callback) {
    bool dataUpdated = BP32.update();
    if(dataUpdated) {
        callback();
    }
}