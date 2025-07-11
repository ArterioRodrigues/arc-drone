#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

typedef void (*Callback)(void);

class Controller {
    public:
        Controller();
        ControllerPtr getController();
        void processController(Callback callback);
    private:
        ControllerPtr controller = nullptr;
        void onConnectedController(BP32Controller* controller);
        void onDisconnectedController(BP32Controller* controller);
}