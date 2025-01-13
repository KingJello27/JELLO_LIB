#pragma once
#include "main.h"

//Conversion Constants
const double wheelDiameter = 3.25;         // Diameter in inches
const double gearRatio = 48.0 / 36.0;      // Gear ratio (driven/driving)

//Conversion Function
extern double inchesToTicks(double inches);

//Angle Normalization
extern double normalizeAngle180(double theta);
extern double normalizeAngle360(double theta);
