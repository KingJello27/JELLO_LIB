#include "main.h"
#include "robot/globals.hpp"

//Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Motors
pros::Motor intakeHooks(-7, pros::MotorGearset::blue);
pros::Motor intakeRollers(-15, pros::MotorGearset::blue);
pros::Motor lift(-12, pros::MotorGearset::red);

//Intake Motor Group
pros::MotorGroup intakeGroup({-7,-15}, pros::MotorGearset::blue);

//Pneumatics
pros::ADIDigitalOut clamp('E');
pros::ADIDigitalOut tipper('G');
pros::ADIDigitalOut raiser('C');

//Drive Motors
pros::Motor leftFront(-18, pros::MotorGearset::blue);
pros::Motor leftMiddle(-14, pros::MotorGearset::blue);
pros::Motor leftBack(-13, pros::MotorGearset::blue);
pros::Motor rightFront(9, pros::MotorGearset::blue);
pros::Motor rightMiddle(4, pros::MotorGearset::blue);
pros::Motor rightBack(21, pros::MotorGearset::blue);

//Drive Motor Groups
pros::MotorGroup leftdr({-18,-14,-13}, pros::MotorGears::blue);
pros::MotorGroup rightdr({9,4,21}, pros::MotorGears::blue);

//Rotation Sensors
pros::Rotation liftRotationSensor(8);

//Color Sensors
pros::Optical intakeColorSensor(10);
pros::Optical clampColorSensor(1);