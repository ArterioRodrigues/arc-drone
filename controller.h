#pragma once
#include <Arduino.h> 
#include <Bluepad32.h>

extern ControllerPtr ps4Controller;

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpController();
void processController();
void controllerSetup();
