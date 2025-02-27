#ifndef Helper_h
#define Helper_h

double mapd(double x, double inMin, double inMax, double outMin, double outMax);

double closestDivisibleBy(double value, int num);

float batteryPercentage(float voltage);

bool isCutoffPercentageReached(float voltage);

#endif