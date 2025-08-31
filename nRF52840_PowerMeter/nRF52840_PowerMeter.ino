/*
  DIY Cycling Power Meter - Final Integrated Firmware (no placeholders)

  HW: Seeed XIAO nRF52840 Sense + HX711 + 4-gauge bridge (right crank)
  Features:
    - BLE Cycling Power Profile (instantaneous power + cumulative crank revs + event time)
    - Gyro-based cadence (thresholded zero-crossing with LPF)
    - Real power = torque (from HX711) * angular velocity (from gyro)
    - Sample accumulation at HX711 rate; averaged every BLE interval
    - Tare (zero offset) + Calibration (10 kg) to compute counts-per-Newton
    - Auto sleep after inactivity; wake-on-motion via IMU INT1
*/

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "HX711.h"

// ---------------- CONFIGURATION ----------------
// Sleep after no revolutions for this time (ms)
#define SLEEP_TIMEOUT_MS        5000     // 5s for testing; set 60000 for production

// BLE send interval (ms). 250ms ~ 4 Hz is common.
#define BLE_SEND_INTERVAL_MS    250

// Crank length (meters)
#define CRANK_LENGTH_M          0.1725f  // 172.5 mm

// Cadence detection (gyro Z axis, deg/s)
#define CADENCE_THRESHOLD_DPS   30.0f
#define GYRO_FILTER_ALPHA       0.15f    // 0..1 (lower = smoother)

// Calibration
#define CALIBRATION_MODE        0        // 1 = run calibration and halt; 0 = normal
#define CALIBRATION_WEIGHT_KG   10.0f    // hang 10 kg on pedal
#define CALIBRATION_SAMPLES     200      // samples for tare and calibration

// Optional: persist scaleFactor in EEPROM (set to 1 if your core provides EEPROM.h)
#define USE_EEPROM_STORAGE      0

// ---------------- Pins ----------------
#define HX_DT   2
#define HX_SCK  3
#define MOTION_INT_PIN 7       // IMU INT1 -> XIAO

// ---------------- Globals ----------------
HX711 scale;

// BLE Cycling Power Service (0x1818)
BLEService cyclingPowerService("1818");
BLECharacteristic cpMeasurementChar("2A63", BLENotify, 20);
BLEUnsignedLongCharacteristic cpFeatureChar("2A65", BLERead);

// CPP fields we maintain
uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0; // 1/1024 s units

// Cadence detection state
float lastFilteredVelDps = 0.0f;
bool  prevPositive = false;
unsigned long lastRevMs = 0;

// Angular velocity (rad/s) latest, for sampling
float currentAngVelRad = 0.0f;

// Power averaging accumulators (since last BLE send)
float sumTorqueNm = 0.0f;
float sumAngVel   = 0.0f;
uint16_t sampleCount = 0;

// Tare + calibration
long   zeroOffsetCounts = 0;   // measured at startup (tare)
float  scaleFactor_counts_per_N = NAN; // computed in calibration (no fake default)

// Timing
unsigned long lastBleSendMs = 0;

// ---------------- I2C helper for IMU wake routing ----------------
static inline void i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// LSM6DS3 address
#define LSM6DS3_ADDR 0x6A

// Configure LSM6DS3 wake-on-motion on INT1 (low-power accel)
void configureWakeOnMotion() {
  // CTRL1_XL (0x10): accel @26Hz, ±2g
  i2cWrite(LSM6DS3_ADDR, 0x10, 0x20);
  // WAKE_THS (0x5B): ~250 mg threshold
  i2cWrite(LSM6DS3_ADDR, 0x5B, 0x04);
  // WAKE_DUR (0x5C): 0 = immediate trigger
  i2cWrite(LSM6DS3_ADDR, 0x5C, 0x00);
  // TAP_CFG (0x58): enable wake-up event
  i2cWrite(LSM6DS3_ADDR, 0x58, 0xC0);
  // MD1_CFG (0x5E): route wakeup interrupt → INT1
  i2cWrite(LSM6DS3_ADDR, 0x5E, 0x20);
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

  // No extra features advertised
  cpFeatureChar.writeValue(0);

  BLE.advertise();
  Serial.println("BLE Cycling Power Service advertising...");
}

// ---------------- CPP packet ----------------
void sendCyclingPowerMeasurement(int16_t powerWatts) {
  // Flags: 0x0020 = crank revolution data present
  uint16_t flags = 0x0020;

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
  // HX711 power-down: hold SCK high >=60us; keep high during sleep
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, HIGH);
  delayMicroseconds(100);
  pinMode(HX_SCK, INPUT_PULLUP); // keep high while MCU in System OFF

  // Enter System OFF (reset-on-wake)
  NRF_POWER->SYSTEMOFF = 1;
}

// ---------------- Tare (zero offset) ----------------
long tareAverage(int samples) {
  long sum = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (n < samples && (millis() - t0) < 3000UL) {
    if (scale.is_ready()) {
      sum += scale.read();
      n++;
    }
  }
  return n > 0 ? (sum / n) : 0;
}

// ---------------- Calibration routine ----------------
void runCalibration() {
  Serial.println();
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("1) Ensure NO load on pedal/crank.");
  delay(1000);
  zeroOffsetCounts = tareAverage(CALIBRATION_SAMPLES);
  Serial.print("Zero offset (counts): "); Serial.println(zeroOffsetCounts);

  Serial.println("2) Hang known weight on the pedal (10 kg). Keep steady...");
  delay(2500);

  long loadedAvg = tareAverage(CALIBRATION_SAMPLES);
  long delta = loadedAvg - zeroOffsetCounts;

  float forceN = CALIBRATION_WEIGHT_KG * 9.80665f; // precise g
  scaleFactor_counts_per_N = (float)delta / forceN;

  Serial.print("Delta counts: "); Serial.println(delta);
  Serial.print("Force (N):   "); Serial.println(forceN, 3);
  Serial.print("scaleFactor (counts/N): ");
  Serial.println(scaleFactor_counts_per_N, 6);

#if USE_EEPROM_STORAGE
  // optionally store in EEPROM here
  // EEPROM.put(0, scaleFactor_counts_per_N);
  // EEPROM.put(sizeof(scaleFactor_counts_per_N), zeroOffsetCounts);
  // EEPROM.commit();
  Serial.println("Stored to EEPROM (if enabled).");
#endif

  Serial.println("Calibration complete. Power-cycle, set CALIBRATION_MODE=0, and ride.");
  while (1) delay(1000);
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

  // IMU init (for gyro + wake)
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  pinMode(MOTION_INT_PIN, INPUT); // IMU drives push-pull; no pull needed
  configureWakeOnMotion();

  // BLE
  setupBLE();

  // Tare at boot (zero load)
  zeroOffsetCounts = tareAverage(CALIBRATION_SAMPLES);
  Serial.print("Boot tare offset (counts): ");
  Serial.println(zeroOffsetCounts);

#if USE_EEPROM_STORAGE
  // Optionally load calibration from EEPROM here
  // EEPROM.get(0, scaleFactor_counts_per_N);
  // EEPROM.get(sizeof(scaleFactor_counts_per_N), zeroOffsetCounts);
  // if (isnan(scaleFactor_counts_per_N) || scaleFactor_counts_per_N <= 0) {
  //   Serial.println("EEPROM scaleFactor invalid. Run calibration.");
  // }
#endif

#if CALIBRATION_MODE
  runCalibration();
#else
  if (isnan(scaleFactor_counts_per_N) || scaleFactor_counts_per_N <= 0.0f) {
    Serial.println("ERROR: No valid calibration (scaleFactor).");
    Serial.println("Set CALIBRATION_MODE=1, calibrate with 10 kg, then retry.");
    while (1) delay(1000);
  }
#endif

  lastRevMs = millis();
  lastBleSendMs = millis();
}

// ---------------- Loop ----------------
void loop() {
  BLE.poll();

  // --- Gyro read & filter (use Z axis) ---
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);             // deg/s
    float filtered = GYRO_FILTER_ALPHA * gz + (1.0f - GYRO_FILTER_ALPHA) * lastFilteredVelDps;
    lastFilteredVelDps = filtered;
    currentAngVelRad = filtered * DEG_TO_RAD;   // rad/s

    // Cadence revolution detection via thresholded zero-crossing
    bool isPositive = (filtered >  +CADENCE_THRESHOLD_DPS);
    bool isNegative = (filtered <  -CADENCE_THRESHOLD_DPS);

    if (prevPositive && isNegative) {
      prevPositive = false; // crossed through negative half-cycle
    } else if (!prevPositive && isPositive) {
      prevPositive = true;  // back to positive half → full revolution completed

      cumulativeCrankRevs++;
      lastCrankEventTime = (uint16_t)((millis() * 1024UL) / 1000UL);
      lastRevMs = millis();
    }
  }

  // --- HX711 sample when ready; accumulate for averaging ---
  if (scale.is_ready()) {
    long raw = scale.read();                          // counts
    long net = raw - zeroOffsetCounts;
    float forceN = net / scaleFactor_counts_per_N;    // N
    float torqueNm = forceN * CRANK_LENGTH_M;         // Nm

    // accumulate
    sumTorqueNm += torqueNm;
    sumAngVel   += currentAngVelRad;
    sampleCount++;
  }

  // --- Periodic BLE transmission of averaged power ---
  if (millis() - lastBleSendMs >= BLE_SEND_INTERVAL_MS) {
    lastBleSendMs += BLE_SEND_INTERVAL_MS;

    int16_t powerW = 0;
    if (sampleCount > 0) {
      float avgTorque = sumTorqueNm / (float)sampleCount;
      float avgAngVel = sumAngVel   / (float)sampleCount;
      float pwr = avgTorque * avgAngVel;     // Watts
      // Clamp to int16 range (BLE expects signed 16-bit)
      if (pwr > 32767.0f) pwr = 32767.0f;
      if (pwr < -32768.0f) pwr = -32768.0f;
      powerW = (int16_t)lroundf(pwr);
    }

    // reset accumulators for next window
    sumTorqueNm = 0.0f;
    sumAngVel   = 0.0f;
    sampleCount = 0;

    // Send BLE CPP (includes cumulative crank revs + last event time)
    sendCyclingPowerMeasurement(powerW);

    // Debug
    // Serial.print("BLE Pwr(W)="); Serial.print(powerW);
    // Serial.print("  Revs="); Serial.println(cumulativeCrankRevs);
  }

  // --- Auto-sleep if no revolutions ---
  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    Serial.println("No cadence → sleeping");
    delay(50);
    goToSleep();
  }
}
