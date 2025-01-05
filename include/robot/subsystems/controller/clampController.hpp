#pragma once
#include "main.h"

//HELPER FUNCTIONS
//Enact Clamp
extern void enactClamp();

//Detect Clamp
extern void detectClamp();

//Toggle
extern void toggleClampButtonPressed();

//Initalize
extern void clampInit();

//State Getter
extern bool getClampState();

//Clamp Async Controller
extern void clampAsyncController(void * param);
