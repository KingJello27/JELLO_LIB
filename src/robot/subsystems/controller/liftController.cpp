#include "robot/subsystems/controller/liftController.hpp"
#include "robot/globals.hpp"
#include "utils/utils.hpp"

double ladyBrownTargetPosition;
double kP;
double error;
double input;
bool settled = false;
bool manual;
int liftCounter;

double getData(){
    return input;
}

//Setters
void setPosition(double targetPosition){
    ladyBrownTargetPosition = thetaToTicks(targetPosition);
}

//Initialization
void liftInit(){
    liftRotationSensor.reset_position();
    ladyBrownTargetPosition = 0;
    kP = 0.02;
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

//Async Controller
void liftAsyncController(void * param){
    while (true){
        if (manual == false){
            settled = false;
            error = ladyBrownTargetPosition - liftRotationSensor.get_position(); 
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
    }
    else if (liftCounter == 1) {
        setPosition(90);
        waitUntilSettled();
        manual = true;
    }
    else if (liftCounter == 2) {
        setPosition(0);
    }
    liftCounter ++;
    liftCounter = liftCounter % 3;
}

void waitUntilSettled(){
    while (settled == false){
        pros::delay(25);
    }
}