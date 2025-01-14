#include "robot/globals.hpp"
#include "robot/subsystems/controller/intakeController.hpp"
#include "robot/subsystems/controller/liftController.hpp"

bool isActuallyIntaking  = true;
double targetVelocity;
double actualVelocity;


//HELPER FUNCTIONS
void setIntake(int power) {
    intakeGroup.move(power);
}
//use intake.move(voltage)
// -127 to 127

//DRIVER CONTROL FUNCTIONS
void setIntakeMotors() {
    if (isActuallyIntaking  == true){
        if (controller.get_digital(DIGITAL_R1)) {
            intakeGroup.move_velocity(200);
            
        } 
        else if (controller.get_digital(DIGITAL_R2)) {
            intakeGroup.move_velocity(-200);
        } 
        else {
            intakeGroup.move_velocity(0);
        }
    }
}

void intakeAsyncController(void * param){
    while(true){
        if (getIsPrimed() == false){
            //Cooldown
            pros::delay(500);

            targetVelocity = intakeHooks.get_target_velocity();
            actualVelocity = intakeHooks.get_actual_velocity();

            if ((targetVelocity > 100) && ((targetVelocity - actualVelocity) > (targetVelocity / 2))) {
                isActuallyIntaking  = false;

                intakeGroup.move(-127);
                pros::delay(1000);

                intakeGroup.move(0);
                isActuallyIntaking  = true;
            }

            pros::delay(10);
        }
    }
}