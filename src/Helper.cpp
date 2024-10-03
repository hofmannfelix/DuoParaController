#include <Arduino.h>

double mapd(double x, double inMin, double inMax, double outMin, double outMax) {
  return (constrain(x, inMin, inMax) - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

double closestDivisibleBy(double value, int num) {
  return (((int)(value / num + 0.5)) * num);
}

float batteryPercentage(float voltage) {
  float battPercent = 0;

  if (voltage > 94.8) {
    battPercent = mapd(voltage, 94.8, 99.6, 90, 100);
  } else if (voltage > 93.36) {
    battPercent = mapd(voltage, 93.36, 94.8, 80, 90);
  } else if (voltage > 91.68) {
    battPercent = mapd(voltage, 91.68, 93.36, 70, 80);
  } else if (voltage > 89.76) {
    battPercent = mapd(voltage, 89.76, 91.68, 60, 70);
  } else if (voltage > 87.6) {
    battPercent = mapd(voltage, 87.6, 89.76, 50, 60);
  } else if (voltage > 85.2) {
    battPercent = mapd(voltage, 85.2, 87.6, 40, 50);
  } else if (voltage > 82.32) {
    battPercent = mapd(voltage, 82.32, 85.2, 30, 40);
  } else if (voltage > 80.16) {
    battPercent = mapd(voltage, 80.16, 82.32, 20, 30);
  } else if (voltage > 78) {
    battPercent = mapd(voltage, 78, 80.16, 10, 20);
  } else if (voltage > 60.96) {
    battPercent = mapd(voltage, 60.96, 78, 0, 10);
  }
  return constrain(battPercent, 0, 100);
}

void decodeCyphalFrameId(uint32_t input, uint8_t &nodeId, uint16_t &subjectId)
{
    // Mask for Node ID: Bits 0-7 (0xFF is 8 bits all set to 1, or 11111111 in binary)
    nodeId = input & 0xFF;

    // Mask for Subject ID: Bits 8-20 (13 bits) -> shift input right by 8 bits, then mask with 0x1FFF (13 bits)
    subjectId = (input >> 8) & 0x1FFF;
}

void decodeBufferCurrentInfo(const unsigned char *buffer, int &electricalSpeed, int &busCurrent, uint16_t &operatingStatus)
{
    // Decode electricalSpeed (2 bytes: buffer[0] and buffer[1])
    electricalSpeed = (static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8)) / 10;

    // Decode busCurrent (2 bytes: buffer[2] and buffer[3])
    busCurrent = (static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8)) / 10;

    // Decode operatingStatus (2 bytes: buffer[4] and buffer[5])
    operatingStatus = static_cast<uint16_t>(buffer[4]) | (static_cast<uint16_t>(buffer[5]) << 8);
}

void decodeBufferVoltageInfo(const unsigned char *buffer, int &outputThrottle, float &busVoltage, int &mosTemp, int &capacitanceTemp, int &motorTemp)
{
    // Decode outputThrottle (2 bytes: buffer[0] and buffer[1])
    outputThrottle = static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);

    // Decode busVoltage (2 bytes: buffer[2] and buffer[3])
    busVoltage = (static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8)) / 10;

    // Decode mosTemp (1 byte: buffer[4])
    mosTemp = buffer[4] - 40;

    // Decode capacitanceTemp (1 byte: buffer[5])
    capacitanceTemp = buffer[5] - 40;

    // Decode motorTemp (1 byte: buffer[6])
    motorTemp = buffer[6] - 40;
}
