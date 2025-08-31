/* --------------------------------------------------
   DIY Cycling Power Meter - Final Unified Sketch
   Hardware: Seeed XIAO nRF52840 Sense + HX711 + strain gauges
   Features:
   - BLE Cycling Power Profile (CPP)
   - Cumulative crank revolutions + timestamps
   - HX711 strain gauge readout
   - Cadence detection via IMU gyro
   - Auto deep sleep after inactivity, wake-on-motion from IMU
   -------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "HX711.h"

// ---------------- CONFIGURATION ----------------
#define SLEEP_TIMEOUT_MS 5000   // 5s for testing, use 60000 for 60s production
#define CADENCE_THRESHOLD_DPS 30     // gyro threshold deg/s
#define GYRO_FILTER_ALPHA 0.1        // LPF smoothing factor

// HX711 pins
#define HX_DT   2
#define HX_SCK  3
HX711 scale;

// IMU (LSM6DS3)
#define LSM6DS3_ADDR 0x6A
#define MOTION_INT_PIN 7   // IMU INT1 -> XIAO

// BLE Cycling Power Service
BLEService cyclingPowerService("1818");
BLECharacteristic cpMeasurementChar("2A63", BLENotify, 20);
BLEUnsignedLongCharacteristic cpFeatureChar("2A65", BLERead);

uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0; // in 1/1024 s units

// Cadence detection
float lastFilteredVel = 0;
bool prevPositive = false;
unsigned long lastRevMs = 0;

// ---------------- I2C helper ----------------
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ---------------- IMU setup ----------------
void configureWakeOnMotion() {
  // Configure IMU wake-on-motion
  writeRegister(0x10, 0x20); // CTRL1_XL: accel @26Hz ±2g
  writeRegister(0x5B, 0x04); // WAKE_THS: ~250 mg
  writeRegister(0x5C, 0x00); // WAKE_DUR: immediate
  writeRegister(0x58, 0xC0); // TAP_CFG: enable wake
  writeRegister(0x5E, 0x20); // MD1_CFG: route INT1
}

// ---------------- BLE setup ----------------
void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }
  BLE.setLocalName("DIY-Powermeter");
  BLE.setAdvertisedService(cyclingPowerService);
  cyclingPowerService.addCharacteristic(cpMeasurementChar);
  cyclingPowerService.addCharacteristic(cpFeatureChar);
  BLE.addService(cyclingPowerService);
  cpFeatureChar.writeValue(0);
  BLE.advertise();
  Serial.println("BLE Cycling Power Service advertising...");
}

// ---------------- Send CPP packet ----------------
void sendCyclingPowerMeasurement(int16_t powerWatts) {
  uint16_t flags = 0x0020; // Crank revolution data present
  uint8_t payload[8];
  int i = 0;

  payload[i++] = lowByte(flags);
  payload[i++] = highByte(flags);
  payload[i++] = lowByte(powerWatts);
  payload[i++] = highByte(powerWatts);
  payload[i++] = lowByte(cumulativeCrankRevs);
  payload[i++] = highByte(cumulativeCrankRevs);
  payload[i++] = lowByte(lastCrankEventTime);
  payload[i++] = highByte(lastCrankEventTime);

  cpMeasurementChar.writeValue(payload, i);
}

// ---------------- HX711 + MCU deep sleep ----------------
void goToSleep() {
  Serial.println("Entering deep sleep...");
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, HIGH);
  delayMicroseconds(100);
  pinMode(HX_SCK, INPUT_PULLUP); // hold high in sleep
  NRF_POWER->SYSTEMOFF = 1;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // HX711 init
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT);
  scale.begin(HX_DT, HX_SCK);
  delay(75);

  // IMU init
  pinMode(MOTION_INT_PIN, INPUT);
  configureWakeOnMotion();
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }

  // BLE init
  setupBLE();

  lastRevMs = millis();
}

// ---------------- Loop ----------------
void loop() {
  BLE.poll();

  // --- Gyro-based cadence detection ---
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // Use Z-axis
    float vel = gz; // deg/s
    float filtered = GYRO_FILTER_ALPHA * vel + (1 - GYRO_FILTER_ALPHA) * lastFilteredVel;
    lastFilteredVel = filtered;

    bool isPositive = (filtered > CADENCE_THRESHOLD_DPS);
    bool isNegative = (filtered < -CADENCE_THRESHOLD_DPS);

    if (prevPositive && isNegative) {
      prevPositive = false;
    }
    else if (!prevPositive && isPositive) {
      prevPositive = true;

      cumulativeCrankRevs++;
      lastCrankEventTime = (uint16_t)((millis() * 1024UL) / 1000UL);
      lastRevMs = millis();

      long raw = scale.read_average(5);
      int16_t powerW = (int16_t)(raw / 1000); // replace with calibration

      sendCyclingPowerMeasurement(powerW);

      Serial.print("Rev ");
      Serial.print(cumulativeCrankRevs);
      Serial.print(" | Power = ");
      Serial.println(powerW);
    }
  }

  // --- Auto sleep on inactivity ---
  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    Serial.println("No activity → sleep");
    delay(50);
    goToSleep();
  }
}
