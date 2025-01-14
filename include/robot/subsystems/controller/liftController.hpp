#pragma once
#include "main.h"

//Setters
extern void setPosition(double targetPosition);

//Initialization
extern void liftInit();

//Async Controller
extern void liftAsyncController(void * param);

extern double getData();

extern void waitUntilSettled();

//Getter
extern bool getIsPrimed();

//Lift Opcontrol
extern void setLift();

//lift manual
extern bool manual;