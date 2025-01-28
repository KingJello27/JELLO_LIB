#include "main.h"

//Quick Wait
extern void setQuickWait(double input);
extern double quickWaitTarget;

//Lateral Movement Functions
extern void moveDistance(double target, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void moveDistance(double target, double maxVoltage, double minVoltage);                                     //Simple Function

//Angular Movement Functions
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap); //Advanced Function
extern void turnAngle(double targetAngle, double maxVoltage, double minVoltage);                                     //Simple Function

//Swing Movement Functions
extern void swingLeft(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap);   //Advanced Function
extern void swingLeft(double targetAngle, double maxVoltage, double minVoltage);                                       //Simple Function

extern void swingRight(double targetAngle, double maxVoltage, double minVoltage, double timeout, double integralCap);  //Advanced Function
extern void swingRight(double targetAngle, double maxVoltage, double minVoltage);                                      //Simple Function