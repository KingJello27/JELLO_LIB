#pragma once
#include "main.h"

//PID Constants
const double latKp = 0, latKi = 0, latKd = 0;
const double angKp = 0, angKi = 0, angKd = 0;
const double swingKp = 0, swingKi = 0, swingKd = 0;

//Drive Initialization
extern void chassisInit();
extern void setMotorsCoast();

//Chassis State
extern bool chassisSettled;
extern double chassisError;

//Getters
extern double getChassisError();
extern bool isChassisSettled();

//User Control
extern void setDriveMode(int input);
extern void setDriveMotors();

//Chassis Movement Function
extern void chassisMotion(double leftVoltage, double rightVoltage);