#include "main.h"
#include "comp/autonomous.hpp"
#include "robot/subsystems/managers/pneumaticsManager.hpp"
#include "robot/subsystems/controller/liftController.hpp"
#include "robot/subsystems/controller/clampController.hpp"
#include "robot/subsystems/controller/chassisController.hpp"
#include "robot/subsystems/controller/intakeController.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(3, "Auton Selection: %s", autonNames[selectionIndex]);
            pros::delay(20);
        }
	});

	//Auton Selector
    pros::lcd::register_btn0_cb(leftShift);
    pros::lcd::register_btn2_cb(rightShift);

	//Init Functions
	pneumaticsInit();
	liftInit();
	clampInit();
	chassisInit();

	//Async Tasks
	pros::Task ladyBrownTask(liftAsyncController);
	pros::Task intakeTask(intakeAsyncController);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
