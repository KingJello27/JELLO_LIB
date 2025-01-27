#include "robot/subsystems/controller/chassisController.hpp"
#include "robot/globals.hpp"
#include "utils/utils.hpp"

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

//Lateral Drive Functions with PID

//Advanced Function
void moveDistance(double target, double maxVoltage, double minVoltage, double timeout, double integralCap){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 1;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = inchesToTicks(target) - ((leftdr.get_position() + rightdr.get_position()) / 2);

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (latKp * error) + (latKi * integral) + (latKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, output);

        pros::delay(updatePeriod);
    }
}

//Simple Function
void moveDistance(double target, double maxVoltage, double minVoltage){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 1;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    double timeout = 4000;
    double integralCap = 15;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = inchesToTicks(target) - ((leftdr.get_position() + rightdr.get_position()) / 2);

        if (abs(error) < integralCap){
            integral = integral + error;
        }else {
            integral = 0;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (latKp * error) + (latKi * integral) + (latKd * derivative);

        previousError = error;
        
        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, output);

        pros::delay(updatePeriod);
    }
}


//Angular Drive Functions with PID

//Advanced Function
void turnAngle(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap){
    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (angKp * error) + (angKi * integral) + (angKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, -output);

        pros::delay(updatePeriod);
    }
}

//Simple Function
void turnAngle(double targetAngle, double maxVoltage, double minVoltage){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    double timeout = 4000;
    double integralCap = 15;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (angKp * error) + (angKi * integral) + (angKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, -output);

        pros::delay(updatePeriod);
    }
}


//Swing Drive Functions with PID

//Advanced Functions
void swingLeft(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap){
    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (swingKp * error) + (swingKi * integral) + (swingKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(0, output);

        pros::delay(updatePeriod);
    }
}

void swingRight(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap){
    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (swingKp * error) + (swingKi * integral) + (swingKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, 0);

        pros::delay(updatePeriod);
    }
}

//Simple Functions
void swingLeft(double targetAngle, double maxVoltage, double minVoltage){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    double timeout = 4000;
    double integralCap = 15;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (swingKp * error) + (swingKi * integral) + (swingKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(0, output);

        pros::delay(updatePeriod);
    }
}

void swingRight(double targetAngle, double maxVoltage, double minVoltage){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    bool settled = false;
    double error = 0;
    double integral = 0;
    double previousError = 0;
    double output = 0;
    double derivative = error;

    double settleError = 2;
    double settleTime = 200;
    double updatePeriod = 10;
    double timeSettled = 0;
    double totalTime = 0;

    double timeout = 4000;
    double integralCap = 15;

    while (!settled){
        
        if (totalTime > timeout && timeout != 0){
            settled = true;
        }else if (timeSettled > settleTime){
            settled = true;
        }else {
            settled = false;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));

        if (abs(error) < integralCap){
            integral = integral + error;
        }

        if (error == 0){
            integral = 0;
        }

        if (error < 0 && previousError > 0 || error > 0 && previousError < 0){
            integral = 0;
        }

        output = (swingKp * error) + (swingKi * integral) + (swingKd * derivative);

        previousError = error;

        derivative = error - previousError;

        if (abs(error) < settleError){
            timeSettled = timeSettled + updatePeriod;
        }else{
            timeSettled = 0;
        }

        if (output > maxVoltage){
            output = maxVoltage;
        }

        if (output < minVoltage){
            output = minVoltage;
        }

        chassisMotion(output, 0);

        pros::delay(updatePeriod);
    }
}
