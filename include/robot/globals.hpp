#pragma once
#include "main.h"

//Controller
extern pros::Controller controller;

//Motors
extern pros::Motor intakeHooks, intakeRollers, lift;

//Intake Motor Group
extern pros::MotorGroup intakeGroup;

//Pneumatics
extern pros::ADIDigitalOut clamp, tipper, raiser;

//Drive Motors
extern pros::Motor leftFront, leftMiddle, leftBack, rightFront, rightMiddle, rightBack;

//Drive Motor Groups
extern pros::MotorGroup leftdr, rightdr;

//IMU
extern pros::IMU imu;

//Rotation Sensors
extern pros::Rotation liftRotationSensor;

//Color Sensors
extern pros::Optical intakeColorSensor;
extern pros::Optical clampColorSensor;