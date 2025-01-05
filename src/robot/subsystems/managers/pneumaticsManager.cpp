#include "main.h"
#include "robot/subsystems/managers/pneumaticsManager.hpp"
#include "robot/globals.hpp"

//Booleans
bool clampState;
bool tipperState;
bool raiserState;

//Initialization
void pneumatcsInit(){
    clampState = false;
    tipperState = false;
    raiserState = false;

    clamp.set_value(clampState);
    tipper.set_value(tipperState);
    raiser.set_value(raiserState);
}

//State Getter
bool getClampState(){
    return clampState;
}

bool getTipperState(){
    return tipperState;
}

bool getRaiserState(){
    return raiserState;
}

//State Togglers
void toggleClamp(){
    clampState = !clampState;
    clamp.set_value(clampState);
}

void toggleTipper(){
    tipperState = !tipperState;
    tipper.set_value(tipperState);
}

void toggleRaiser(){
    raiserState = !raiserState;
    raiser.set_value(raiserState);
}
