#include <Arduino.h>
#include "globals.h"

double mapd(double x, double inMin, double inMax, double outMin, double outMax) {
  return (constrain(x, inMin, inMax) - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

double closestDivisibleBy(double value, int num) {
  return (((int)(value / num + 0.5)) * num);
}

float batteryPercentage(float voltage) {
  float battPercent = 0;
  voltage /= BATTERY_CELLS;

  if (voltage > 3.95) { // 94.8 / 24
    battPercent = mapd(voltage, 3.95, 4.15, 90, 100); // 94.8-99.6 / 24
  } else if (voltage > 3.89) { // 93.36 / 24
    battPercent = mapd(voltage, 3.89, 3.95, 80, 90); // 93.36-94.8 / 24
  } else if (voltage > 3.82) { // 91.68 / 24
    battPercent = mapd(voltage, 3.82, 3.89, 70, 80); // 91.68-93.36 / 24
  } else if (voltage > 3.74) { // 89.76 / 24
    battPercent = mapd(voltage, 3.74, 3.82, 60, 70); // 89.76-91.68 / 24
  } else if (voltage > 3.65) { // 87.6 / 24
    battPercent = mapd(voltage, 3.65, 3.74, 50, 60); // 87.6-89.76 / 24
  } else if (voltage > 3.55) { // 85.2 / 24
    battPercent = mapd(voltage, 3.55, 3.65, 40, 50); // 85.2-87.6 / 24
  } else if (voltage > 3.43) { // 82.32 / 24
    battPercent = mapd(voltage, 3.43, 3.55, 30, 40); // 82.32-85.2 / 24
  } else if (voltage > 3.34) { // 80.16 / 24
    battPercent = mapd(voltage, 3.34, 3.43, 20, 30); // 80.16-82.32 / 24
  } else if (voltage > 3.25) { // 78 / 24
    battPercent = mapd(voltage, 3.25, 3.34, 10, 20); // 78-80.16 / 24
  } else if (voltage > 2.54) { // 60.96 / 24
    battPercent = mapd(voltage, 2.54, 3.25, 0, 10); // 60.96-78 / 24
  }
  return constrain(battPercent, 0, 100);
}

bool isCutoffPercentageReached(float voltage) {
  return batteryPercentage(voltage) < 0.1 * CUTOFF_BATTERY_PERCENTAGE;
}
