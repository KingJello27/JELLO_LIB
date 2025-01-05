#pragma once
#include "main.h"

//Initialization
extern void pneumaticsInit();

//State Getter
extern bool getTipperState();
extern bool getRaiserState();

//State Togglers
extern void toggleTipper();
extern void toggleRaiser();

