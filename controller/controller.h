#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

//left stick X        ctl->axisX() 
//left stick Y        ctl->axisY() 
//right stick X        ctl->axisRX() 
//right stick Y        ctl->axisRY() 
//
//L2                  ctl->brake()
//R2                  ctl->throttle()
//Cross               BUTTON_A
//Circle              BUTTON_B
//Square              BUTTON_X
//Triangle            BUTTON_Y
//L1                  BUTTON_SHOULDER_L
//R1                  BUTTON_SHOULDER_R
//L2                  BUTTON_TRIGGER_L
//R2                  BUTTON_TRIGGER_R
//
//UP                  DPAD_UP
//DOWN                DPAD_DOWN
//LEFT                DPAD_LEFT
//RIGHT               DPAD_RIGHT
//
//if (ctl->buttons() & BUTTON_A) {
//if (ctl->dpad() & DPAD_UP) {
//int throttle = ctl->axisY();  // -512 (forward) to 511 (back)

typedef void (*Callback)(void);
ControllerPtr _controller;
void onConnectedController(ControllerPtr controller);
void onDisconnectedController(ControllerPtr controller);

namespace Drone {
    class Controller {
        public:
            Controller();
            void setup();
            ControllerPtr getController();
            void processController(Callback callback);
    };
};
