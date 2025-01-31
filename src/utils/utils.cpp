#include "utils/utils.hpp"
#include "robot/globals.hpp"

//Conversion Functions
double inchesToTicks(double inches){
    double wheelCircumference = M_PI * wheelDiameter;
    double ticksPerWheelRev = 360 * driveGearRatio;
    double ticksPerInch = ticksPerWheelRev / wheelCircumference;

    return (inches * ticksPerInch);
}

double thetaToTicks(double theta){
    return (theta * liftGearRatio);
}

double voltsToPower(double voltage){
    return (voltage * (127/12000));
}

double powerToVolts(double power){
    return (power * (12000/127));
}

//Angle Normalization
double normalizeAngle180(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

double normalizeAngle360(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}
