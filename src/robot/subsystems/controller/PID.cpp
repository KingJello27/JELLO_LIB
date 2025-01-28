#include "robot/subsystems/controller/PID.hpp"
#include "robot/subsystems/controller/chassisController.hpp"
#include "robot/globals.hpp"
#include "utils/utils.hpp"

//Quick Wait
double quickWaitTarget = 0;

void setQuickWait(double input){
    quickWaitTarget = input;
}

//Lateral Drive Functions with PID

//Advanced Function
void moveDistance(double target, double maxVoltage, double minVoltage, double timeout, double integralCap){

    leftdr.tare_position();
    rightdr.tare_position();
    leftdr.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightdr.set_brake_mode(MOTOR_BRAKE_HOLD);

    chassisSettled = false;
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

    if (quickWaitTarget > 0){
        target = target + quickWaitTarget;
    }

    while (!chassisSettled){

        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = inchesToTicks(target) - ((leftdr.get_position() + rightdr.get_position()) / 2);
        chassisError = error;

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

        if (abs(error) < inchesToTicks(settleError)){
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

    chassisSettled = false;
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

    if (quickWaitTarget > 0){
        target = target + quickWaitTarget;
    }

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = inchesToTicks(target) - ((leftdr.get_position() + rightdr.get_position()) / 2);
        chassisError = error;

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

        if (abs(error) < inchesToTicks(settleError)){
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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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

    chassisSettled = false;
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

    while (!chassisSettled){
        
        if (totalTime > timeout && timeout != 0){
            chassisSettled = true;
        }else if (timeSettled > settleTime){
            chassisSettled = true;
        }else {
            chassisSettled = false;
        }

        if (quickWaitTarget > 0){
            if (abs(error) < 5)
            chassisSettled = true;
        }

        error = normalizeAngle180(targetAngle - normalizeAngle360(imu_sensor.get_heading()));
        chassisError = error;

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


