#ifndef EscData_h
#define EscData_h

#include <CircularBuffer.hpp>

class EscData {
  static const int SHUTOFF_RPM_DIFF = 200;
  static const int MIN_RPM_FOR_DIFF_SHUTOFF = 500; // min rpm to reach to trigger diff checker
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
  CircularBuffer<int, 2> escRpmBuffer;
  unsigned long lastRpmUpdate;
  uint16_t operatingStatus;

  EscData();

  // returns if average rpm change is diverging by SHUTOFF percentage
  bool isRpmChangeToLarge(EscData &escData); 
  bool isLastRpmUpdateExceeded();
  int avgRpm();
};

#endif