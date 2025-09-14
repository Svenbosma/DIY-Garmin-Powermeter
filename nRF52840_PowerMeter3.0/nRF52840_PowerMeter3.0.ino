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

// ---------- IMU ----------
LSM6DS3 myIMU(I2C_MODE, 0x6A);
float lastAccelX = 0.0f;
bool prevAccelPositive = false;

// ---------- BLE ----------
BLEDis bledis;

// Cycling Power Service
BLEService cyclingPowerService = BLEService(0x1818);
BLECharacteristic cpMeasurementChar = BLECharacteristic(0x2A63);
BLECharacteristic cpFeatureChar = BLECharacteristic(0x2A65);

// NUS UART Service using BLEUart
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
void goToDeepSleep();
void setupWakeOnMotion();
void goToSystemOff();
void handleUART();
void processUARTCommand(String cmd);

// ---------- HX711 ISR ----------
void hx711ISR() { hx711ReadyFlag = true; }

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to initialize
  logPrintln("DIY Powermeter v3.1 start");

  Wire.begin();

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

// ---------- loop ----------
void loop() {
  handleUART();  // BLE UART handling

  // Idle until HX711 ready
  hx711ReadyFlag = false;
  if (!hx711ReadyFlag) { __SEV(); __WFE(); }

  if (hx711ReadyFlag || scale.is_ready()) {
    long raw = scale.read();
    long net = raw - zeroOffsetCounts;

    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N;
      float torqueNm = forceN * CRANK_LENGTH_M;
      sumTorqueNm += torqueNm;
      torqueSampleCount++;
    }

    // Cadence detection (forward only)
    float ax = myIMU.readFloatAccelX();
    float gx = myIMU.readFloatGyroX();
    bool forwardPedal = (gx > GYRO_FORWARD_THRESHOLD);
    bool accelPositive = (ax > ACCEL_THRESHOLD_G);

    if (!prevAccelPositive && accelPositive && forwardPedal) {
      unsigned long now = millis();
      float dt = (now - lastRevMs) / 1000.0f;
      if (dt > 0.2f) { // ignore too short revs
        cumulativeCrankRevs++;
        lastCrankEventTime = (uint16_t)((now * 1024UL)/1000UL);
        float angVelRad = (2.0f * PI / dt);
        processRevolution(angVelRad);
        lastRevMs = now;
      }
    }
    prevAccelPositive = accelPositive;
  }
  /*
  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    logPrintln("No pedaling -> deep sleep");
    delay(50);
    goToSystemOff();
  }
  */
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

  // ----- NUS UART -----
  bleUart.begin();   // BLEUart handles Notify + Write automatically

  // ----- Advertising -----
  Bluefruit.Advertising.addService(cyclingPowerService);
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start();

  logPrintln("BLE advertising started (Cycling Power + UART ready)");
}

// ---------- UART Handling ----------
void handleUART() {
  while (bleUart.available()) {
    String cmd = bleUart.readString();
    processUARTCommand(cmd);
  }
}

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

void doTare() {
  logPrintln("Tare: remove load from pedal");
  delay(1000);
  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);
  logPrint("zeroOffsetCounts = "); logPrintln(String(zeroOffsetCounts));
}

void runCalibration(float knownMassKg) {
  logPrintln("=== Calibration Start ===");
  doTare();
  logPrintln("Step 1: Tare complete.");
  logPrint("Step 2: Hang "); logPrint(String(knownMassKg)); logPrintln(" kg and press any key...");

  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read();

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

// ---------- SYSTEMOFF / Wake-on-motion ----------
void setupWakeOnMotion() {
  // 416Hz, ±2g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL9_XL, 0x07);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x08);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x01);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
}

void goToSystemOff() {
  digitalWrite(HX_SCK, HIGH);      // HX711 sleep

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);

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
