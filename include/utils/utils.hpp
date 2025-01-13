#pragma once
#include "main.h"

//Conversion Constants
const double wheelDiameter = 3.25;         // Diameter in inches
const double driveGearRatio = 48.0 / 36.0;      // Gear ratio (driven/driving)
const double liftGearRatio = 5 / 1;

//Conversion Function
extern double inchesToTicks(double inches);
extern double thetaToTicks(double theta);

//Angle Normalization
extern double normalizeAngle180(double theta);
extern double normalizeAngle360(double theta);
