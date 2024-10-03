#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h> // smoothing for throttle
#include <ESP32Servo.h> 
#include <Thread.h> // run tasks at different intervals
#include <StaticThreadController.h>
#include <CircularBuffer.hpp>
#include <ezButton.h>

#define ESC_PWM_PIN A0
#define ESC_RPM_PIN A3
#define THROTTLE_PIN A2
#define BUTTON_PIN D4
#define ESC_CAN_RX_PIN D7

#define MOTOR_POLE_MAGNETS 15

#define CURRENT_INFO_CAN_ID 6160
#define VOLTAGE_INFO_CAN_ID 6161

#define ESC_DISARMED_PWM 1010
#define ESC_MIN_PWM 1030  // ESC min is 1050
#define ESC_MAX_PWM 1990  // ESC max 1950
#define ANALOG_READ_MAX 4090
#define POT_READ_MAX 1400
#define POT_MIN_OFFSET 0  // Pot value delta to actually accelerate to make it less touchy
#define POT_OUT_OF_BOUNDS_VALUE 200 // Stop throttle if values are out of throttle limits (for safety)
#define BATT_MIN_V 49.0  // 7S min (use 42v for 6S)
#define BATT_MAX_V 58.8  // 7S max (use 50v for 6S)
#define INITIALIZED_THRESHOLD 10 // set initial value after pot value is stable (delta < 10)

#define BLE_CONNECTION_THREASHOLD 10000
#define BLE_CONNECTED_THREAD_INTERVAL 1000
#define BLE_CONNECTING_THREAD_INTERVAL 50

// Telemetry
class BleData {
  public:
  float volts;
  float temperatureC;
  float amps;
  float rpm;
  float kW;
  float usedKwh;
  float batteryPercentage;
  float power;
};

class EscData {
  public:
  float busVoltage;
  int busCurrent;
  int mosTemp;
  int capacitanceTemp;
  int motorTemp;
  int outputThrottle;
  int electricalSpeed;
  int wattage;
  int rpm;
  uint16_t operatingStatus;
};