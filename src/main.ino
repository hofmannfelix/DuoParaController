#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h>
#include <Thread.h>
#include <StaticThreadController.h>
#include <ezButton.h>
#include <WiFi.h>
#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include "mcp_can.h"
#include "globals.h"
#include "CruiseControl.h"
#include "Helper.h"
#include "MadCyphal.h"
#include "StringPrinter.h"
#include "OtaUpdater.h"
#include "EscData.h"

// Power Switch
ezButton powerSwitch(BUTTON_PIN);
bool isArmed = false;

// Throttle
ResponsiveAnalogRead pot(THROTTLE_PIN, false);
CircularBuffer<int, 19> potBuffer;
CruiseControl cruiseControl;
auto startTime = millis();
int prevPotLvl = 0;
int initialPotLvl = -1;
int pwmSignal = 0;

// ESC
CircularBuffer<float, 50> voltageBuffer;
MCP_CAN CAN(ESC_CAN_RX_PIN);
EscData leftEscData;
EscData rightEscData;

// Bluetooth Low Energy Service
auto bleConnectedTime = millis();
auto bleThreadInterval = BLE_CONNECTING_THREAD_INTERVAL;
StringPrinter bleSerial;
OtaUpdater otaUpdater;
BleData bleData = BleData();
BLEService bleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEDevice central;
BLEDoubleCharacteristic batteryCharacteristic("00000000-0019-b100-01e8-f2537e4f6c00", BLERead | BLENotify);
BLEDoubleCharacteristic voltageCharacteristic("00000000-0019-b100-01e8-f2537e4f6c01", BLERead | BLENotify);
BLEDoubleCharacteristic temperatureCharacteristic("00000000-0019-b100-01e8-f2537e4f6c02", BLERead | BLENotify);
BLEDoubleCharacteristic ampsCharacteristic("00000000-0019-b100-01e8-f2537e4f6c03", BLERead | BLENotify);
BLEDoubleCharacteristic rpmCharacteristic("00000000-0019-b100-01e8-f2537e4f6c04", BLERead | BLENotify);
BLEDoubleCharacteristic kWCharacteristic("00000000-0019-b100-01e8-f2537e4f6c05", BLERead | BLENotify);
BLEDoubleCharacteristic usedKwhCharacteristic("00000000-0019-b100-01e8-f2537e4f6c06", BLERead | BLENotify);
BLEDoubleCharacteristic powerCharacteristic("00000000-0019-b100-01e8-f2537e4f6c07", BLERead | BLENotify);
BLEBoolCharacteristic armedCharacteristic("00000000-0019-b100-01e8-f2537e4f6c08", BLERead | BLENotify);
BLEBoolCharacteristic cruiseCharacteristic("00000000-0019-b100-01e8-f2537e4f6c09", BLERead | BLENotify);
BLEDoubleCharacteristic altitudeCharacteristic("00000000-0019-b100-01e8-f2537e4f6c10", BLEWriteWithoutResponse | BLENotify);
BLECharacteristic logCharacteristic("00000000-0020-b100-01e8-f2537e4f6c00", BLERead | BLENotify, 20);
BLECharacteristic otaCharacteristic("00000000-0020-b100-01e8-f2537e4f6c01", BLERead | BLEWrite | BLENotify, 20);

Thread powerSwitchThread = Thread();
Thread throttleThread = Thread();
Thread telemetryThread = Thread();
Thread trackPowerThread = Thread();
Thread bleThread = Thread();
Thread bleLogThread = Thread();
Thread otaThread = Thread();
StaticThreadController<7> threads(&powerSwitchThread, &throttleThread, &telemetryThread, &trackPowerThread, &bleThread, &bleLogThread, &otaThread);

void setup()
{
    Serial.begin(115200);
    Watchdog.enable(4000);

    // setup power switch button
    powerSwitch.setDebounceTime(50);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    powerSwitchThread.onRun(handlePowerSwitch);
    powerSwitchThread.setInterval(22);

    // setup Throttle
    analogReadResolution(12);
    pot.setAnalogResolution(ANALOG_READ_MAX);
    throttleThread.onRun(handleThrottle);
    throttleThread.setInterval(22);

    // setup ESC
    while (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

    trackPowerThread.onRun(handlePowerTracking);
    trackPowerThread.setInterval(250);

    telemetryThread.onRun(handleTelemetry);
    telemetryThread.setInterval(1);

    // setup BLE
    if (!BLE.begin())
    {
        bleSerial.println("Starting Bluetooth@ Low Energy module failed!");
        while (1);
    }
    BLE.setLocalName("SP-140");
    BLE.setAdvertisedService(bleService);
    bleService.addCharacteristic(batteryCharacteristic);
    bleService.addCharacteristic(temperatureCharacteristic);
    bleService.addCharacteristic(voltageCharacteristic);
    bleService.addCharacteristic(ampsCharacteristic);
    bleService.addCharacteristic(rpmCharacteristic);
    bleService.addCharacteristic(kWCharacteristic);
    bleService.addCharacteristic(usedKwhCharacteristic);
    bleService.addCharacteristic(powerCharacteristic);
    bleService.addCharacteristic(armedCharacteristic);
    bleService.addCharacteristic(cruiseCharacteristic);
    bleService.addCharacteristic(altitudeCharacteristic);
    bleService.addCharacteristic(logCharacteristic);
    bleService.addCharacteristic(otaCharacteristic);

    BLE.addService(bleService);
    BLE.advertise();

    bleThread.onRun(handleBleData);
    bleThread.setInterval(bleThreadInterval);
    bleLogThread.onRun(handleBleLog);
    bleLogThread.setInterval(100);
    otaThread.onRun(handleOta);
    otaThread.setInterval(1000);
}

void loop()
{
    threads.run();
}

void handlePowerSwitch()
{
    powerSwitch.loop();
    auto armed = powerSwitch.getState() == LOW;
    static auto lastSwitchOff = millis();
    static bool isSwitchOn = armed;
    bool hasSwitched = armed != isSwitchOn;
    if (hasSwitched)
    {
        isSwitchOn = armed;
        if (!armed)
            lastSwitchOff = millis();
    }
    auto isQuickToggled = millis() - lastSwitchOff < 500;

    if (hasSwitched && armed && isQuickToggled && cruiseControl.hasRequiredAltitude())
    {
        cruiseControl.enable();
        bleSerial.println("cruise enabled");
    }

    if (armed)
    {
        if (!isArmed)
            bleSerial.println("armed");
        isArmed = true;
    }
    else if (!isQuickToggled)
    {
        if (isArmed)
            bleSerial.println("disarmed");
        isArmed = false;
    }
}

void handleThrottle()
{
    Watchdog.reset();

    pot.update();
    int potRaw = ANALOG_READ_MAX - pot.getValue();
    int potLvl = 0;
    for (auto i = 0; i < potBuffer.size(); i++)
    {
        potLvl += potBuffer[i] / potBuffer.size();
    }
    potBuffer.push(potRaw);

    if (millis() - startTime < 2000) return;

    static auto lastUpdate = millis();
    if (prevPotLvl != potLvl && millis() - lastUpdate > 1000)
    {
        lastUpdate = millis();
        bleSerial.print("pwmSignal: ");
        bleSerial.print(pwmSignal);
        bleSerial.print(" min max: ");
        bleSerial.print(initialPotLvl);
        bleSerial.print(" ");
        bleSerial.print(initialPotLvl + POT_READ_MAX);
        bleSerial.print(" potLvl: ");
        bleSerial.print(potLvl);
        bleSerial.print(" rawPotLvl: ");
        bleSerial.println(potRaw);

        bleSerial.print("throttle: ");
        bleSerial.println(leftEscData.outputThrottle);

        static auto maxLastRpmUpdate = 0;
        if (millis() - leftEscData.lastRpmUpdate > maxLastRpmUpdate) {
            bleSerial.print("Max last update l: ");
            bleSerial.println(millis() - leftEscData.lastRpmUpdate);
            maxLastRpmUpdate = millis() - leftEscData.lastRpmUpdate;
        }
        if (millis() - rightEscData.lastRpmUpdate > maxLastRpmUpdate) {
            bleSerial.print("Max last update r: ");
            bleSerial.println(millis() - rightEscData.lastRpmUpdate);
            maxLastRpmUpdate = millis() - rightEscData.lastRpmUpdate;
        }
    }

    // set initial potentiometer Lvl once value changes are less than INITIALIZED_THRESHOLD
    auto isInitialized = isArmed && (initialPotLvl != -1 || abs(potLvl - prevPotLvl) < INITIALIZED_THRESHOLD);
    potLvl = limitedThrottle(potLvl, prevPotLvl, 120);

    if (isInitialized && initialPotLvl == -1)
    {
        initialPotLvl = potLvl;
        bleSerial.print("init to potLvl: ");
        bleSerial.println(potLvl);
    }

    if (isArmed && isInitialized && !isCutoffPercentageReached(bleData.volts))
    {
        // calculate pwm signal relative to the initial potentiometer Lvl
        pwmSignal = mapd(potLvl - POT_MIN_OFFSET, initialPotLvl, initialPotLvl + POT_READ_MAX, ESC_MIN_PWM, ESC_MAX_PWM);
        bool isPotWithinBounds = constrain(potLvl, initialPotLvl - POT_OUT_OF_BOUNDS_VALUE, initialPotLvl + POT_READ_MAX + POT_OUT_OF_BOUNDS_VALUE) == potLvl;
        bool isRpmChangeToLarge = leftEscData.isRpmChangeToLarge(rightEscData);
        bool isRpmUpdateExceeded = leftEscData.isLastRpmUpdateExceeded() || rightEscData.isLastRpmUpdateExceeded();

        if (isPotWithinBounds) // && !isRpmChangeToLarge && !isRpmUpdateExceeded
        {
            static auto cruiseInitializedTimestamp = millis();
            if (cruiseControl.isEnabled() && !cruiseControl.isInitialized())
            {
                cruiseControl.initialize(pwmSignal);
                cruiseInitializedTimestamp = millis();
                bleSerial.print("initial cruise pwm: ");
                bleSerial.println(pwmSignal);
            }
            // disable cruise control if trigger held more than 4 seconds and more then 20% after enabling it
            if (pwmSignal > ESC_MIN_PWM + (ESC_MAX_PWM - ESC_MIN_PWM) * 0.2 && millis() - cruiseInitializedTimestamp > 4000)
            {
                if (cruiseControl.isEnabled()) bleSerial.println("cruise disabled");
                cruiseControl.disable();
            }
            if (cruiseControl.isInitialized())
            {
                writePwm(cruiseControl.calculateCruisePwm());
            }
            else
            {
                writePwm(pwmSignal);
            }
        }
        // else
        // {
            if (isRpmChangeToLarge) {
                bleSerial.print("RPM limit: l: ");
                bleSerial.print(leftEscData.avgRpm());
                bleSerial.print(", r: ");
                bleSerial.println(rightEscData.avgRpm());
            }
            if (isRpmUpdateExceeded) {
                bleSerial.print("RPM update exceeded limit: l: ");
                bleSerial.print(leftEscData.lastRpmUpdate);
                bleSerial.print(", r: ");
                bleSerial.println(rightEscData.lastRpmUpdate);    
            }
        //     writePwm(ESC_MIN_PWM);
        // }
        
        // else {
        //     if (isRpmChangeToLarge) bleSerial.println("RPM change exceeded limit");
        //     if (isRpmUpdateExceeded) bleSerial.println("RPM update exceeded limit");
        //     writePwm(ESC_MIN_PWM);
        // }
    }
    else
    {
        if (!isArmed)
        {
            if (cruiseControl.isEnabled())
                bleSerial.println("cruise disabled");
            initialPotLvl = -1;
            cruiseControl.disable();
        }
        writePwm(ESC_DISARMED_PWM);
    }
}

void handlePowerTracking()
{
    static unsigned long prevPwrMillis = 0;
    unsigned long currentPwrMillis = millis();
    unsigned long msec_diff = (currentPwrMillis - prevPwrMillis); // eg 0.30 sec
    prevPwrMillis = currentPwrMillis;

    if (isArmed)
    {
        leftEscData.wattsHoursUsed += round(leftEscData.wattage / 60 / 60 * msec_diff) / 1000.0;
        rightEscData.wattsHoursUsed += round(rightEscData.wattage / 60 / 60 * msec_diff) / 1000.0;
    }
}

void handleTelemetry()
{
    unsigned long canId;
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);
        canId = CAN.getCanId();
    }
    uint8_t nodeId;
    uint16_t subjectId;
    decodeCyphalFrameId(canId, nodeId, subjectId);

    if (nodeId != ESC_LEFT_CAN_ID && nodeId != ESC_RIGHT_CAN_ID) return;
    auto &escData = nodeId == ESC_LEFT_CAN_ID ? leftEscData : rightEscData;

    voltageBuffer.push(escData.busVoltage);
    escData.wattage = escData.busCurrent * escData.busVoltage;

    switch (subjectId)
    {
    case CURRENT_INFO_CAN_ID:
        decodeBufferCurrentInfo(buf, escData.electricalSpeed, escData.busCurrent, escData.operatingStatus);
        escData.escRpmBuffer.push(escData.electricalSpeed * (60 / MOTOR_POLE_MAGNETS));
        escData.lastRpmUpdate = millis();
        break;
    case VOLTAGE_INFO_CAN_ID:
        int capacitanceTemp, motorTemp;
        decodeBufferVoltageInfo(buf, escData.outputThrottle, escData.busVoltage, escData.mosTemp, capacitanceTemp, motorTemp);
        break;
    }
}

void handleBleLog()
{
    auto chunk = bleSerial.readChunk();
    logCharacteristic.writeValue(chunk.c_str(), chunk.length());
}

void handleOta()
{
    if (isArmed) return;

    int length = otaCharacteristic.valueLength();
    char otaValue[length + 1];
    otaCharacteristic.readValue(otaValue, length);
    otaValue[length] = '\0';

    static std::string updateUrl;
    if (strcmp(otaValue, "UPDATE") == 0)
    {
        bleSerial.println("OTA activated");
        otaUpdater.activate();
        updateUrl = "";
    }
    auto newUpdatUrl = otaUpdater.getUpdateUrl();
    if (!newUpdatUrl.empty() && updateUrl != newUpdatUrl)
    {
        otaCharacteristic.writeValue((updateUrl = newUpdatUrl).c_str(), newUpdatUrl.length());
    }
}

void handleBleData()
{
    // change thread interval if BLE is connected vs connecting so connection process is faster
    auto isBleConnectionThresholdReached = millis() - bleConnectedTime > BLE_CONNECTION_THREASHOLD;
    if (!isBleConnectionThresholdReached && bleThreadInterval != BLE_CONNECTING_THREAD_INTERVAL)
    {
        bleThread.setInterval(bleThreadInterval = BLE_CONNECTING_THREAD_INTERVAL);
    }
    else if (isBleConnectionThresholdReached && bleThreadInterval != BLE_CONNECTED_THREAD_INTERVAL)
    {
        bleThread.setInterval(bleThreadInterval = BLE_CONNECTED_THREAD_INTERVAL);
    }

    if (!central || !central.connected())
    {
        bleConnectedTime = millis();
        central = BLE.central();
        return;
    }

    static EscData *lastEsc = nullptr;
    auto &escData = lastEsc == &leftEscData ? rightEscData : leftEscData;
    lastEsc = &escData;
    
    bleData.volts = escData.busVoltage;
    bleData.batteryPercentage = batteryPercentage(smoothedBatteryVoltage());
    bleData.amps = escData.busCurrent;
    bleData.temperatureC = escData.mosTemp;
    bleData.rpm = escData.avgRpm();
    bleData.kW = constrain(escData.wattage / 1000.0, 0, 50);
    bleData.usedKwh = escData.wattsHoursUsed / 1000; 
    bleData.power = mapd(max(pwmSignal, cruiseControl.calculateCruisePwm()), ESC_MIN_PWM, ESC_MAX_PWM, 0, 100);

    // write values
    batteryCharacteristic.writeValue(bleData.batteryPercentage);
    voltageCharacteristic.writeValue(bleData.volts);
    temperatureCharacteristic.writeValue(bleData.temperatureC);
    rpmCharacteristic.writeValue(bleData.rpm);
    ampsCharacteristic.writeValue(bleData.amps);
    usedKwhCharacteristic.writeValue(bleData.usedKwh);
    kWCharacteristic.writeValue(bleData.kW);
    powerCharacteristic.writeValue(bleData.power);
    armedCharacteristic.writeValue(isArmed);
    cruiseCharacteristic.writeValue(cruiseControl.isEnabled());

    // read values
    cruiseControl.setCurrentAltitude(altitudeCharacteristic.value());
}

/// region Helper functions

void writePwm(int pwm) {
  auto frameId = encodeCyphalFrameId(CAN_PRIO_HIGH, BROADCAST_NODE_ID, SEND_PWM_CAN_ID);
  uint16_t throttleValue[] = {static_cast<uint16_t>(pwm), static_cast<uint16_t>(pwm), static_cast<uint16_t>(pwm), static_cast<uint16_t>(pwm)};
  uint8_t encodedThrottle[8] = {0};
  encodeThrottleValues(throttleValue, encodedThrottle);
  CAN.sendMsgBuf(frameId, 1, 8, encodedThrottle);
}

// throttle easing function based on threshold/performance mode
int limitedThrottle(int current, int last, int threshold)
{
    if (current - last >= threshold)
    { // accelerating too fast. limit
        int limitedThrottle = last + threshold;
        prevPotLvl = limitedThrottle; // save for next time
        return limitedThrottle;
    }
    else if (last - current >= threshold * 2)
    {                                               // decelerating too fast. limit
        int limitedThrottle = last - threshold * 2; // double the decel vs accel
        prevPotLvl = limitedThrottle;               // save for next time
        return limitedThrottle;
    }
    prevPotLvl = current;
    return current;
}

float smoothedBatteryVoltage()
{
    float avg = 0.0;

    if (voltageBuffer.isEmpty())
    {
        return avg;
    }
    using index_t = decltype(voltageBuffer)::index_t;
    for (index_t i = 0; i < voltageBuffer.size(); i++)
    {
        avg += voltageBuffer[i] / voltageBuffer.size();
    }
    return avg;
}