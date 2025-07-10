#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

class Controller {
    public:
        Controller();
        void dumpController();
        
    private:
        onConnectedController(ControllerPtr controller);
        onDisconnectedController(ControllerPtr controller);
        ControllerPtr controller;
}
