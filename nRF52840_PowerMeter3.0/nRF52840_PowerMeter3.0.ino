/*
  DIY Powermeter v3.1
  - HX711 interrupt-driven torque sensor
  - BLE Cycling Power Service + UART calibration/tare
  - Cadence from BMI270/LSM6DS3: accel X zero-cross + gyro X sign
  - Wake-on-motion to exit SYSTEMOFF (using double tap)
*/

#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include "LSM6DS3.h"
#include "nrf_gpio.h"

// ---------- pins ----------
#define HX_DT           2
#define HX_SCK          3
#define MOTION_INT_PIN  PIN_LSM6DS3TR_C_INT1   // Not used directly; wake handled by SYSTEMOFF + IMU INT1

// ---------- config ----------
#define CRANK_LENGTH_M      0.1725f
#define ACCEL_THRESHOLD_G   0.05f
#define SLEEP_TIMEOUT_MS    60000UL
#define TARE_SAMPLES        200
#define CAL_SAMPLES         200
#define GYRO_FORWARD_THRESHOLD 0.0f   // positive gx = forward
#define LOW_BATT_THRESHOLD 20    // % at which LED_RED blinks
#define NUM_BATT_SAMPLES 10      // ADC samples to average
#define BATTERY_UPDATE_INTERVAL 6000UL // 6 seconds
#define BLINK_INTERVAL 300       // ms
#define NUM_BLINKS 3             // blinks per battery update

// ---------- globals ----------
HX711 scale;
long zeroOffsetCounts = 0;
float scaleFactor_counts_per_N = NAN;

uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0;

volatile bool hx711ReadyFlag = false;
unsigned long lastRevMs = 0;

float sumTorqueNm = 0.0f;
uint16_t torqueSampleCount = 0;

unsigned long lastBattMs = 0;    // last battery update
unsigned long lastBlinkMs = 0;   // timing for LED blink
uint8_t battPercent = 100;       // current battery percentage
uint8_t blinkCount = 0;          // blink count
bool ledState = false;           // LED state

// ---------- IMU ----------
LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool prevAccelPositive = false;

// ---------- BLE ----------
BLEDis bledis;

// --- Cycling Power Service ---
BLEService cyclingPowerService = BLEService(0x1818);
BLECharacteristic cpMeasurementChar = BLECharacteristic(0x2A63);
BLECharacteristic cpFeatureChar = BLECharacteristic(0x2A65);

// ----- Battery Service -----
BLEService batteryService = BLEService(0x180F);
BLECharacteristic batteryLevelChar = BLECharacteristic(0x2A19);

// -- NUS UART Service using BLEUart --
BLEUart bleUart;

// ---------- helper: log (mirrors Serial -> BLE UART when connected) ----------
void logPrint(const String &msg) {
  Serial.print(msg);
  if (Bluefruit.connected()) bleUart.print(msg);
}
void logPrintln(const String &msg) {
  Serial.println(msg);
  if (Bluefruit.connected()) bleUart.println(msg);
}

// Convenience helpers for const char* and numeric values
void logPrint(const char *msg) { logPrint(String(msg)); }
void logPrintln(const char *msg) { logPrintln(String(msg)); }
void logPrint(long v) { logPrint(String(v)); }
void logPrintln(long v) { logPrintln(String(v)); }
void logPrint(unsigned long v) { logPrint(String(v)); }
void logPrintln(unsigned long v) { logPrintln(String(v)); }
void logPrint(int v) { logPrint(String(v)); }
void logPrintln(int v) { logPrintln(String(v)); }
void logPrint(float v, int digits=2) { logPrint(String(v, digits)); }
void logPrintln(float v, int digits=2) { logPrintln(String(v, digits)); }

// ---------- prototypes ----------
void hx711ISR();
void setupBLE();
long averageCounts(int n, unsigned long timeoutMs);
void doTare();
void runCalibration(float knownMassKg);
void processRevolution(float angVelRad);
void sendCyclingPowerMeasurement(int16_t powerWatts);
void setupWakeOnMotion();
void goToSystemOff();
void handleUART();
void processUARTCommand(String cmd);

// ---------- HX711 ISR ----------
void hx711ISR() {hx711ReadyFlag = true;}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {}
  logPrintln("DIY Powermeter v3.1 start");

  Wire.begin();

  nrf_gpio_cfg_output(13);   // P0.13
  nrf_gpio_pin_clear(13);    // LOW = 100 mA
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP);
  scale.begin(HX_DT, HX_SCK);
  delay(75);
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

  if (myIMU.begin() != 0) {
    logPrintln("IMU init failed!");
    while(1);
  }

  setupBLE();
  doTare();
  lastRevMs = millis();

  logPrintln("Ready. Commands via UART: 'c'=calib, 't'=tare, 'm <kg>'=custom calib.");
}

// ---------- main loop ----------
void loop() {
    handleUART();  // BLE UART handling

    // Process HX711 if ready
    if (hx711ReadyFlag || scale.is_ready()) {
        hx711ReadyFlag = false;

        long raw = scale.read();
        long net = raw - zeroOffsetCounts;

        if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
            float forceN = (float)net / scaleFactor_counts_per_N;
            float torqueNm = forceN * CRANK_LENGTH_M;
            sumTorqueNm += torqueNm;
            torqueSampleCount++;
        }

        // Cadence detection (forward pedal only)
        float ax = myIMU.readFloatAccelX();
        float gx = myIMU.readFloatGyroX();
        bool forwardPedal = (gx > GYRO_FORWARD_THRESHOLD);
        bool accelPositive = (ax > ACCEL_THRESHOLD_G);

        if (!prevAccelPositive && accelPositive && forwardPedal) {
            logPrintln("Good job, you are spinning");
            unsigned long now = millis();
            float dt = (now - lastRevMs) / 1000.0f;
            if (dt > 0.2f) {
                cumulativeCrankRevs++;
                lastCrankEventTime = (uint16_t)((now * 1024UL) / 1000UL);
                float angVelRad = (2.0f * PI / dt);
                processRevolution(angVelRad);
                lastRevMs = now;
            }
        }
        prevAccelPositive = accelPositive;
    }

    // Deep sleep if no pedaling
    if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
        logPrintln("No pedaling -> deep sleep");
        goToSystemOff();
    }

    // Battery check and LED blink
    batteryCheckAndLED();
}

// ---------- BLE ----------
void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DIY-Powermeter");

  // Device Info
  bledis.setManufacturer("DIY");
  bledis.setModel("Powermeter");
  bledis.begin();

  // ----- Cycling Power Service -----
  cyclingPowerService.begin();
  cpMeasurementChar.setProperties(CHR_PROPS_NOTIFY);
  cpMeasurementChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpMeasurementChar.setFixedLen(8);
  cpMeasurementChar.begin();

  cpFeatureChar.setProperties(CHR_PROPS_READ);
  cpFeatureChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpFeatureChar.setFixedLen(4);
  cpFeatureChar.begin();
  uint32_t features = 0;
  cpFeatureChar.write(&features, sizeof(features));

  // ----- Battery Service -----
  batteryService.begin();
  batteryLevelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryLevelChar.setFixedLen(1);
  batteryLevelChar.begin();
  uint8_t initialBatt = 100;
  batteryLevelChar.write8(initialBatt);

  // ----- NUS UART -----
  bleUart.begin();

  // ----- Advertising -----
  Bluefruit.Advertising.addService(cyclingPowerService);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start();

  logPrintln("BLE advertising started (Cycling Power + UART + Battery ready)");
}

// --- Low battery voltage handler ----
void batteryCheckAndLED() {
  unsigned long now = millis();

  // 1. Check if 60s passed to read battery
  if (now - lastBattMs >= BATTERY_UPDATE_INTERVAL) {
    lastBattMs = now;
    battPercent = readBatteryPercent();
    batteryLevelChar.notify8(battPercent);
    logPrint("Battery: "); logPrintln(battPercent);

    blinkCount = 0;     // reset blink for this update
    ledState = false;
    digitalWrite(LED_RED, HIGH); // LED off
    lastBlinkMs = now;
  }

  // 2. Handle low battery LED blink non-blocking (active-low)
  if (battPercent <= LOW_BATT_THRESHOLD && blinkCount < NUM_BLINKS) {
    if (now - lastBlinkMs >= BLINK_INTERVAL) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_RED, ledState ? LOW : HIGH); // invert: LOW=on, HIGH=off

      if (!ledState) blinkCount++; // count only after LED goes off
    }
  } else if (battPercent > LOW_BATT_THRESHOLD) {
    // Ensure LED is off if battery is okay
    digitalWrite(LED_RED, HIGH); // off
  }
}

// ---- Read battery percentage ----
uint8_t readBatteryPercent() {
  float vbatSum = 0.0f;
  for (int i = 0; i < NUM_BATT_SAMPLES; i++) {
    int raw = analogRead(PIN_VBAT);
    float voltage = raw * (3.6f / 1023.0f); // adjust if 12-bit ADC
    vbatSum += voltage;
  }
  float vbatAvg = vbatSum / NUM_BATT_SAMPLES;

  int percent = (int)((vbatAvg - 3.0f) * 100.0f / (4.2f - 3.0f));
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  return percent;
}

// ---------- UART Handling ----------
void handleUART() {
  while (bleUart.available()) {
    String cmd = bleUart.readString();
    processUARTCommand(cmd);
  }
}

// ---- one of many uart handlers ----
void processUARTCommand(String cmd) {
  cmd.trim();
  logPrint("UART command received: "); logPrintln(cmd);

  if (cmd.equalsIgnoreCase("c")) runCalibration(10.0f);
  else if (cmd.equalsIgnoreCase("t")) doTare();
  else if (cmd.startsWith("m")) {
    float kg = cmd.substring(1).toFloat();
    if (kg > 0.0f) runCalibration(kg);
    else logPrintln("Usage: m <mass_kg>");
  } else {
    logPrint("Unknown UART command: "); logPrintln(cmd);
  }
}

// ---------- HX711 helpers ----------
long averageCounts(int n, unsigned long timeoutMs) {
  long sum = 0; int k=0;
  unsigned long t0 = millis();
  while (k<n && (millis()-t0)<timeoutMs) {
    if (scale.is_ready()) { sum+=scale.read(); k++; }
  }
  return (k>0) ? (sum/k) : 0L;
}

// ---- UART input handler (without delay) ----
bool waitForInput(String &cmd, unsigned long timeoutMs = 30000) {
    unsigned long start = millis();
    cmd = "";

    while ((millis() - start) < timeoutMs) {
        // Check USB Serial
        if (Serial.available()) {
            cmd = Serial.readStringUntil('\n');
            cmd.trim();
            return true;
        }

        // Check BLE UART
        if (bleUart.available()) {
            cmd = bleUart.readStringUntil('\n');
            cmd.trim();
            return true;
        }

        // Small delay to prevent busy loop
        delay(10);
    }

    return false; // timeout
}

// ---- Tare ----
void doTare() {
  logPrintln("Tare: remove load from pedal");
  delay(1000);
  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);
  logPrint("zeroOffsetCounts = "); logPrintln(String(zeroOffsetCounts));
}

// --- Run calibration of wheatstone bridge ---
void runCalibration(float knownMassKg) {
  logPrintln("=== Calibration Start ===");
  doTare();
  logPrintln("Step 1: Tare complete.");
  logPrint("Step 2: Hang "); logPrint(String(knownMassKg)); logPrintln(" kg and press any key...");

    String dummy;
    if (!waitForInput(dummy, 120000)) { // wait max 60s
        logPrintln("No input received, aborting...");
        return;
    }

  long loadedAvg = averageCounts(CAL_SAMPLES, 5000);
  long deltaCounts = loadedAvg - zeroOffsetCounts;
  const float g = 9.80665f;
  float forceN = knownMassKg * g;

  if (forceN <= 0.0f || deltaCounts <= 0) {
    logPrintln("Calibration failed");
    return;
  }

  scaleFactor_counts_per_N = (float)deltaCounts / forceN;

  logPrint("deltaCounts = "); logPrintln(String(deltaCounts));
  logPrint("forceN = "); logPrintln(String(forceN,6));
  logPrint("scaleFactor_counts_per_N = "); logPrintln(String(scaleFactor_counts_per_N,6));
  logPrintln("=== Calibration Done ===");
}

// ---------- Revolution / Power ----------
void processRevolution(float angVelRad) {
  float torqueNm = (torqueSampleCount>0) ? (sumTorqueNm/torqueSampleCount) : 0.0f;
  torqueSampleCount = 0; sumTorqueNm = 0.0f;

  float powerW = torqueNm * angVelRad;
  sendCyclingPowerMeasurement((int16_t)powerW);
}

// ---- BLE Cycling power service handler ----
void sendCyclingPowerMeasurement(int16_t powerWatts) {
  uint8_t buf[8] = {0};
  buf[0] = 0b00000100; // flags: crank revolution data present
  buf[1] = lastCrankEventTime & 0xFF;
  buf[2] = (lastCrankEventTime >> 8) & 0xFF;
  buf[3] = cumulativeCrankRevs & 0xFF;
  buf[4] = (cumulativeCrankRevs >> 8) & 0xFF;
  buf[5] = powerWatts & 0xFF;
  buf[6] = (powerWatts >> 8) & 0xFF;
  buf[7] = 0;
  cpMeasurementChar.notify(buf, 8);
}

// ------ SYSTEMOFF / Wake-on-motion -------
void setupWakeOnMotion() {
  // 416Hz, ±2g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL9_XL, 0x07);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x08);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x01);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
}

// ------ Go to sleep -------
void goToSystemOff() {
  digitalWrite(HX_SCK, HIGH);      // HX711 sleep

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  setupWakeOnMotion();

  uint8_t dummy;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);

  pinMode(MOTION_INT_PIN, INPUT_PULLDOWN);
  delay(50);

  nrf_gpio_cfg_sense_input(
    g_ADigitalPinMap[MOTION_INT_PIN],
    NRF_GPIO_PIN_PULLDOWN,
    NRF_GPIO_PIN_SENSE_HIGH
  );
  delay(50);

  logPrintln("Entering SYSTEMOFF...");
  NRF_POWER->SYSTEMOFF = 1;
  // will reset on wake
}
