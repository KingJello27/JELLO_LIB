#include "robot/subsystems/controller/liftController.hpp"
#include "robot/globals.hpp"
#include "utils/utils.hpp"

double liftTargetPosition;
double kP;
double error;
double input;
bool settled = false;
bool manual;
int liftCounter;
bool isPrimed = false;

double getData(){
    return input;
}

//Setters
void setPosition(double targetPosition){
    liftTargetPosition = thetaToTicks(targetPosition);
}

//getters
bool getIsPrimed(){
    return isPrimed;
}

//Initialization
void liftInit(){
    liftRotationSensor.reset_position();
    liftTargetPosition = 0;
    kP = 0.02;
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

//Async Controller
void liftAsyncController(void * param){
    while (true){
        if (manual == false){
            settled = false;
            error = liftTargetPosition - liftRotationSensor.get_position(); 
            if (error < 3 && error > -3){
                error = 0;
                settled = true;
            }
            input = kP * error;
            lift.move_voltage(input * 120);
            pros::delay(25);
        }
    }
}

void setLift(){
    if (liftCounter == 0) {
        setPosition(10);
        isPrimed == true;
    }
    else if (liftCounter == 1) {
        setPosition(90);
        isPrimed == false;
    }
    else if (liftCounter == 2) {
        setPosition(0);
        isPrimed == false;
    }
    liftCounter ++;
    liftCounter = liftCounter % 3;
}

void waitUntilSettled(){
    while (settled == false){
        pros::delay(25);
    }
}