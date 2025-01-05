#pragma once
#include "main.h"

//Initialization
extern void pneumatcsInit();

//State Getter
extern bool getClampState();
extern bool getTipperState();
extern bool getRaiserState();

//State Togglers
extern void toggleClamp();
extern void toggleTipper();
extern void toggleRaiser();

