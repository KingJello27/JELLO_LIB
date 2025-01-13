#include "main.h"
#include "robot/subsystems/managers/pneumaticsManager.hpp"
#include "robot/globals.hpp"
#include "robot/subsystems/controller/liftController.hpp"
#include "robot/subsystems/controller/clampController.hpp"
#include "robot/subsystems/managers/intakeManager.hpp"
#include "robot/subsystems/controller/chassisController.hpp"

void opcontrol() {

	setMotorsCoast();
	
	while (true) {
		
		//Pneumatics
		if (controller.get_digital_new_press(DIGITAL_L1)){
			toggleClampButtonPressed();
		}

		if (controller.get_digital_new_press(DIGITAL_UP)){
			toggleTipper();
		}

		if (controller.get_digital_new_press(DIGITAL_A)){
			toggleRaiser();
		}

		//Lift Control
		if (controller.get_digital_new_press(DIGITAL_L2)){
			setLift();
		}

		//Intake Control
		setIntakeMotors();

		pros::delay(20); 
	}
}