#pragma once
#include "main.h"

//PID Constants
const double latKp = 0, latKi = 0, latKd = 0;
const double angKp = 0, angKi = 0, angKd = 0;
const double swingKp = 0, swingKi = 0, swingKd = 0;

//Drive Initialization
extern void chassisInit();
extern void setMotorsCoast();

//Getters
double getChassisError();
bool isChassisSettled();

//User Control
extern void setDriveMotors();

//Chassis Movement Function
extern void chassisMotion(double leftVoltage, double rightVoltage);

//Lateral Movement Functions
extern void moveDistance(double target, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void moveDistance(double target, double maxVoltage, double minVoltage);                                     //Simple Function

//Angular Movement Functions
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage);                                     //Simple Function

//Swing Movement Functions
extern void swingLeft(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap);   //Advanced Function
extern void swingLeft(double targetAngle, double maxVoltage, double minVoltage);                                       //Simple Function

extern void swingRight(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap);  //Advanced Function
extern void swingRight(double targetAngle, double maxVoltage, double minVoltage);                                      //Simple Function