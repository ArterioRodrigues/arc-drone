#include "controller.h"
#include <functional>

Controller::onConnectedController(ControllerPtr controller) {
    this->controller = controller;
}

Controller::onDisconnectedController(ControllerPtr controller) {
    this->controller = nullptr;
}

Controller::Controller(){
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* address = BP32.localBdAddress();
    Serial.printf("BD Address: %2X:%2X:%2X:%2X:%2X:%2X\n", address[0], address[1], address[2], address[3], address[4], address[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

Controller::processController(std::function<void()> callback) {
    bool dataUpdated = BP32.update();
    if(dataUpdated && this->controller && this->controller.isConnected() && this->controller->hasData()){
        callback()
    }
}
