#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h>
#include <ESP32Servo.h> 
#include <Thread.h>
#include <StaticThreadController.h>
#include <CircularBuffer.hpp>
#include <ezButton.h>

#define THROTTLE_PIN A1
#define BUTTON_PIN D4
#define ESC_CAN_RX_PIN D7

#define ESC_LEFT_CAN_ID 16
#define ESC_RIGHT_CAN_ID 17

#define MOTOR_POLE_MAGNETS 15

#define CURRENT_INFO_CAN_ID 6160
#define VOLTAGE_INFO_CAN_ID 6161

#define ESC_DISARMED_PWM 0
#define ESC_MIN_PWM 0  // ESC min is 0 via CAN
#define ESC_MAX_PWM 2000  // ESC max 2000 via CAN
#define ANALOG_READ_MAX 4096
#define POT_READ_MAX 1400
#define POT_MIN_OFFSET 0  // Pot value delta to actually accelerate to make it less touchy
#define POT_OUT_OF_BOUNDS_VALUE 200 // Stop throttle if values are out of throttle limits (for safety)
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
