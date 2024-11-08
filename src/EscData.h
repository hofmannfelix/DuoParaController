#ifndef EscData_h
#define EscData_h

#include <CircularBuffer.hpp>

class EscData {
  static const int SHUTOFF_RPM_DIFF_PERCENT = 20;
  static const int SHUTOFF_LAST_UPDATE_MS = 200;

  public:
  float busVoltage;
  int busCurrent;
  int mosTemp;
  int capacitanceTemp;
  int motorTemp;
  int outputThrottle;
  int electricalSpeed;
  int wattage;
  float wattsHoursUsed = 0;
  CircularBuffer<int, 10> escRpmBuffer;
  unsigned long lastRpmUpdate;
  uint16_t operatingStatus;

  EscData();

  // returns if average rpm change is diverging by SHUTOFF percentage
  bool isRpmChangeToLarge(EscData &escData); 
  bool isLastRpmUpdateExceeded();
};

#endif