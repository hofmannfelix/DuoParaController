#ifndef Helper_h
#define Helper_h

double mapd(double x, double inMin, double inMax, double outMin, double outMax);

double closestDivisibleBy(double value, int num);

float batteryPercentage(float voltage);

void decodeCyphalFrameId(uint32_t input, uint8_t &nodeId, uint16_t &subjectId);

void decodeBufferCurrentInfo(const unsigned char *buffer, int &electricalSpeed, int &busCurrent, uint16_t &operatingStatus);

void decodeBufferVoltageInfo(const unsigned char *buffer, int &outputThrottle, float &busVoltage, int &mosTemp, int &capacitanceTemp, int &motorTemp);

#endif