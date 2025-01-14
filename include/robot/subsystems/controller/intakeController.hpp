#pragma once
#include "main.h"

//HELPER FUNCTIONS
void setIntake(int velocity);

//DRIVER CONTROL FUNCTIONS
void setIntakeMotors();

//Async Controller
extern void intakeAsyncController(void * param);

//Velocity
extern double actualVelocity;
extern double targetVelocity;