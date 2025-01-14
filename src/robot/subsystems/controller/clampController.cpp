#include "robot/subsystems/controller/clampController.hpp"
#include "robot/globals.hpp"

bool isClamped = true;
bool justChanged = false;
int elapsedClampTime = 0;

//Initialize
void clampInit(){
    clamp.set_value(true);
    isClamped = true;
    justChanged = false;
    elapsedClampTime = 0;
}

//Getter
bool getClampState(){
    return isClamped;
}

//Setter
void setClamp(bool input){
    clamp.set_value(input);
}

void toggleClamp(){
    isClamped = !isClamped;
    clamp.set_value(isClamped);
}

void autoClamp(){
    if(goalSensor.get() < 20 && !justChanged){
        toggleClamp();
        justChanged = true;
        elapsedClampTime = 0;
    }

    if (elapsedClampTime > 1000){
        justChanged = false;
    }
}
