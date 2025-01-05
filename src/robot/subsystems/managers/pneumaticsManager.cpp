#include "main.h"
#include "robot/subsystems/managers/pneumaticsManager.hpp"
#include "robot/globals.hpp"

//Booleans
bool tipperState;
bool raiserState;

//Initialization
void pneumaticsInit(){
    tipperState = false;
    raiserState = false;

    tipper.set_value(tipperState);
    raiser.set_value(raiserState);
}

//State Getter
bool getTipperState(){
    return tipperState;
}

bool getRaiserState(){
    return raiserState;
}

//State Togglers

void toggleTipper(){
    tipperState = !tipperState;
    tipper.set_value(tipperState);
}

void toggleRaiser(){
    raiserState = !raiserState;
    raiser.set_value(raiserState);
}
