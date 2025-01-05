#include "main.h"
#include "robot/subsystems/controller/intakeColorController.hpp"
#include "robot/globals.hpp"

//VARIABLES
bool sawRing = false;
const int bufferTime = 70; //60
const int stopTime = 650;
std::string sortingColorTreshold = "blue";

int ringDetect() {
    float lowestValue = 1000;
    float currColor;
    float revPos = 0.0;
    while(true) {
        currColor = intakeColorSensor.get_hue();

        // DEBUGGING STATEMENTS
        if(currColor < lowestValue) {
            lowestValue = currColor;
        }
        
        if (sortingColorTreshold == "red") {
            if (currColor <= 20) {
                sawRing = true;
            }
        }
        else if (sortingColorTreshold == "blue") {
            if (currColor >= 170) {
                sawRing = true;
            }
        }

        if(sawRing) {
            revPos = 0;
            pros::delay(bufferTime);
            // intakeHooks.spinFor(fwd, .3 ,rev);
            // revPos = intakeHooks.position(rev);
            intakeHooks.move_velocity(0);
            pros::delay(stopTime);
            sawRing = false;
        }

        pros::delay(20);
    }
    return -1;
}

//HELPER FUNCTIONS
bool getSawRing() {
    return sawRing;
}
void setTreshold(std::string color) {
    sortingColorTreshold = color;
}

std::string getTreshold() {
    return sortingColorTreshold;
}