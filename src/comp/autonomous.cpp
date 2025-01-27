#include "main.h"
#include "robot/subsystems/controller/chassisController.hpp"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
    moveDistance(24, 100, 60, 4000, 15);
    pros::delay(10);
    moveDistance(-24, 100, 60);
    pros::delay(10);
    turnAngle(90, 127, 60, 4000, 10);
    pros::delay(10);
    turnAngle(-90, 127, 60);
    pros::delay(10);
    turnAngle(0, 127, 60);
    pros::delay(10);
    swingLeft(-90, 127, 60, 4000, 10);
    pros::delay(10);
    swingRight(0, 127, 60, 4000, 10);
    pros::delay(10);
    swingRight(90, 127, 60);
    pros::delay(10);
    swingLeft(0, 127, 60);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
