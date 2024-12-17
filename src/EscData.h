#ifndef EscData_h
#define EscData_h

#include <CircularBuffer.hpp>

class EscData {
  static const int SHUTOFF_RPM_DIFF_PERCENT = 10;
  static const int MIN_RPM_FOR_DIFF_SHUTOFF = 1000; // min rpm to reach to trigger diff checker
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
  CircularBuffer<int, 3> escRpmBuffer;
  unsigned long lastRpmUpdate;
  uint16_t operatingStatus;

  EscData();

  // returns if average rpm change is diverging by SHUTOFF percentage
  bool isRpmChangeToLarge(EscData &escData); 
  bool isLastRpmUpdateExceeded();
  int avgRpm();
};

#endif