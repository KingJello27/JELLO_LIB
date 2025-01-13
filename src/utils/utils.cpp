#include "utils/utils.hpp"
#include "robot/globals.hpp"

//Conversion Function
double inchesToTicks(double inches){
    double wheelCircumference = M_PI * wheelDiameter;
    double ticksPerWheelRev = 360 * gearRatio;
    double ticksPerInch = ticksPerWheelRev / wheelCircumference;

    return (inches * ticksPerInch);
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
