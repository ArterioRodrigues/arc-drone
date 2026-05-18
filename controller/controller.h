#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

typedef void (*Callback)(void);
ControllerPtr _controller;
void onConnectedController(ControllerPtr controller);
void onDisconnectedController(ControllerPtr controller);

namespace drone {
    class Controller {
        public:
            Controller();
            void setup();
            ControllerPtr getController();
            void processController(Callback callback);
    };
};
