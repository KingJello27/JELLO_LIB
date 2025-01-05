#include "main.h"
#include "robot/globals.hpp"

bool isTilted = false;
bool isGoalDetected = false;
bool isClampButtonPressed = false;
bool clampState;
int sensedColor = 0;

void toggleClampButtonPressed(){
    isClampButtonPressed = !isClampButtonPressed;
}

void detectClamp(){

    sensedColor = clampColorSensor.get_hue();

    if (sensedColor >= 200 && sensedColor <= 280){
        isGoalDetected = true;
    }else{
        isGoalDetected = false;
    }

}

void enactClamp(){

    if (isGoalDetected == true){
        if (controller.get_digital_new_press(DIGITAL_L1)){
            clamp.set_value(false);
        }else{
            clamp.set_value(true);
        }
    }else if (isGoalDetected == false){
        if (controller.get_digital_new_press(DIGITAL_L1)){
            clamp.set_value(true);
        }else{
            clamp.set_value(false);
        }
    }

}

void clampInit(){
    clampState = false;
    clamp.set_value(clampState);
}

bool getClampState(){
    return clampState;
}

//Clamp Async Controller
void clampAsyncController(void * param){
   while(true){
        detectClamp();
        enactClamp();
   } 
}