#include "main.h"
#include "robot/subsystems/managers/intakeManager.hpp"
#include "robot/globals.hpp"
#include "robot/subsystems/controller/intakeColorController.hpp"

//HELPER FUNCTIONS
//velocity goes from -12V to 12V
void setIntake(int voltage) {
    intakeGroup.move_voltage(voltage);
}

//DRIVER CONTROL FUNCTIONS
void setIntakeMotors() {
    if (!getSawRing()) {
        if (controller.get_digital_new_press(DIGITAL_R1)) {
            setIntake(12000);
        }
        else if (controller.get_digital_new_press(DIGITAL_R2)) {
            setIntake(-12000);
        }
        else {
            setIntake(0);
        }
    }
}

// void setIntakeMotors() {
//   int intakePower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
//   setIntake(intakePower);
// }