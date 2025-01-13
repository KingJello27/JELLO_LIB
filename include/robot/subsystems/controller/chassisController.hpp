#pragma once
#include "main.h"

//PID Constants
const double latKp = 0, latKi = 0, latKd = 0;
const double angKp = 0, angKi = 0, angKd = 0;

//Drive Initialization
extern void chassisInit();

//Getters
double getChassisError();
bool isChassisSettled();

//Chassis Movement Function
extern void chassisMotion(double leftVoltage, double rightVoltage);

//Lateral Movement Functions
extern void moveDistance(double target, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void moveDistance(double target, double maxVoltage, double minVoltage);                                     //Simple Function

//Angular Movement Functions
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage);                                     //Simple Function
