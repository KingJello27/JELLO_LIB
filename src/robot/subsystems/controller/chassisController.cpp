#include "robot/subsystems/controller/chassisController.hpp"
#include "robot/globals.hpp"
#include "utils/utils.hpp"

//Chassis Variable
bool chassisSettled;
double chassisError;

//Initialization
void chassisInit(){
    imu_sensor.reset();
    while(imu_sensor.is_calibrating()){
        pros::delay(20);
    }
    controller.rumble("--");
}

void setMotorsCoast(){
    leftdr.set_brake_mode(MOTOR_BRAKE_COAST);
    rightdr.set_brake_mode(MOTOR_BRAKE_COAST);
}

//Getters
bool isChassisSettled(){
    return chassisSettled;
}

double getChassisError(){
    return chassisError;
}

//Chassis Movement Function
void chassisMotion(double leftVoltage, double rightVoltage){
    leftdr.move_voltage(leftVoltage);
    rightdr.move_voltage(rightVoltage);
}

//User Control Drive Functions
int driveMode;

void setDriveMode(int input){
    driveMode = input;
}

void setDriveMotors(){

    //Tank Drive
    int leftInput;
    int rightInput;

    //Arcade Drive
    int latInput;
    int angInput;

    if (driveMode == 0){

        //Tank Drive
        leftInput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        rightInput = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (abs(leftInput) < 10){
            leftInput = 0;
        }

        if (abs(rightInput) < 10){
            rightInput = 0;
        }

        leftInput = powerToVolts(leftInput);
        rightInput = powerToVolts(rightInput);

        chassisMotion(leftInput, rightInput);

    }else if (driveMode == 1){

        //Arcade Drive
        latInput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        angInput = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (abs(latInput) < 10){
            latInput = 0;
        }

        if (abs(angInput) < 10){
            angInput = 0;
        }

        latInput = powerToVolts(latInput);
        angInput = powerToVolts(angInput);

        chassisMotion(latInput + angInput, latInput - angInput);

    }
}


