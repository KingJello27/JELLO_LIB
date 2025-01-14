#pragma once
#include "main.h"

//Toggle
extern void toggleClamp();

//Initalize
extern void clampInit();

//State Getter
extern bool getClampState();

//State Setter
extern void setClamp(bool input);

//Auto Clamp
extern void autoClamp();
extern bool justChanged;
extern int elapsedClampTime;
