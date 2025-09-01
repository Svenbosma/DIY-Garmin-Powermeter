/* 
  DIY Cycling Power Meter - Final Flashable Firmware
  - Sends one Cycling Power Measurement (0x2A63) per crank revolution
  - Proper calibration helper (serial), tare, per-rev averaging of HX711
  - IMU gyro for cadence / revolution detection
  - Auto deep sleep, wake-on-motion via IMU INT1
  - BLE format conforms to Cycling Power Service: Crank Revolution Data Present flag set
*/

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "HX711.h"

// ---------------- PINS & HW ----------------
#define HX_DT           2
#define HX_SCK          3
#define MOTION_INT_PIN  7     // IMU INT1 -> XIAO (wake)

// ---------------- CONFIG ----------------
#define CRANK_LENGTH_M          0.1725f   // 172.5 mm crank (m)
#define CADENCE_THRESHOLD_DPS   30.0f     // threshold for +/-
#define GYRO_FILTER_ALPHA       0.15f     // LPF alpha (0..1)
#define SLEEP_TIMEOUT_MS        60000UL   // deep sleep after no revs (ms)
#define TARE_SAMPLES            200
#define CAL_SAMPLES             200

// ---------------- GLOBALS ----------------
HX711 scale;
long zeroOffsetCounts = 0;               // tare
float scaleFactor_counts_per_N = NAN;    // counts per Newton (calibration)

uint32_t cumulativeCrankRevs = 0;        // internal (can be >16bit); we'll send low 16 bits
uint16_t lastCrankEventTime = 0;         // 1/1024s units for BLE

// cadence detection
float lastFilteredGz = 0.0f;
bool prevPositive = false;
unsigned long lastRevMs = 0;

// per-rev HX711 accumulation
float sumTorqueNm = 0.0f;
uint16_t torqueSampleCount = 0;

// BLE objects
BLEService cyclingPowerService("1818");
BLECharacteristic cpMeasurementChar("2A63", BLENotify, 8); // flags(2) + inst power(2) + crank revs(2) + last event time(2)
BLEUnsignedLongCharacteristic cpFeatureChar("2A65", BLERead);

// IMU I2C address
#define LSM6DS3_ADDR 0x6A

// ---------------- I2C helper ----------------
static inline void i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ---------------- Forward declarations ----------------
void setupBLE();
void configureWakeOnMotion();
long averageCounts(int n, unsigned long timeoutMs);
void doTare();
void runCalibration(float knownMassKg);
void sendCyclingPowerMeasurement(int16_t powerWatts);
void goToSleep();
void processRevolution(float angVelRad);

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

  // IMU init (Gyro + wake)
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  pinMode(MOTION_INT_PIN, INPUT); // IMU drives INT1
  configureWakeOnMotion();

  // BLE init
  setupBLE();

  // Tare at boot to get zero offset
  doTare();

  lastRevMs = millis();
  Serial.println("DIY Powermeter ready.");
  Serial.println("Commands: 'c' = calibrate (10kg), 't' = tare, 'm <kg>' = calibrate with custom mass");
}

// ---------------- Loop (clean sections) ----------------
void loop() {
  BLE.poll();

  // --- Serial commands (calibration / tare) ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("c")) {
      runCalibration(10.0f);    // 10 kg default
      lastRevMs = millis();
    } else if (cmd.equalsIgnoreCase("t")) {
      doTare();                 // redo tare
      lastRevMs = millis();
    } else if (cmd.startsWith("m")) {
      // format: m 7.5
      float kg = cmd.substring(1).toFloat();
      if (kg > 0.0f) {
        runCalibration(kg);
        lastRevMs = millis();
      } else {
        Serial.println("Usage: m <mass_kg>");
      }
    }
  }

  // --- Read gyro & detect crank revolution ---
  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz); // deg/s

    // filter
    float filtered = GYRO_FILTER_ALPHA * gz + (1.0f - GYRO_FILTER_ALPHA) * lastFilteredGz;
    lastFilteredGz = filtered;

    bool isPositive = (filtered > +CADENCE_THRESHOLD_DPS);
    bool isNegative = (filtered < -CADENCE_THRESHOLD_DPS);

    if (prevPositive && isNegative) {
      // crossed into negative half
      prevPositive = false;
    } else if (!prevPositive && isPositive) {
      // positive crossing again => one full revolution completed
      prevPositive = true;

      // compute angular velocity (rad/s) from lastRevMs
      unsigned long now = millis();
      float dt = (now - lastRevMs) / 1000.0f;
      float angVelRad = (dt > 0.0f) ? (2.0f * PI / dt) : 0.0f;

      // update BLE counters
      cumulativeCrankRevs++;
      lastCrankEventTime = (uint16_t)((now * 1024UL) / 1000UL);

      // compute averaged torque & power and send BLE
      processRevolution(angVelRad);

      lastRevMs = now;
    }
  }

  // --- Sample HX711 whenever ready (accumulate torque between revolutions) ---
  if (scale.is_ready()) {
    long raw = scale.read();            // counts
    long net = raw - zeroOffsetCounts;  // apply tare

    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N; // N
      float torqueNm = forceN * CRANK_LENGTH_M;            // Nm
      sumTorqueNm += torqueNm;
      torqueSampleCount++;
    }
  }

  // --- Auto-sleep if no pedaling for a while ---
  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    Serial.println("Inactivity timeout, going to deep sleep...");
    delay(50);
    goToSleep();
  }
}

// ---------------- FUNCTIONS ----------------

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

  // Advertise that we have no optional features in cpFeatureChar (0)
  cpFeatureChar.writeValue(0);

  BLE.advertise();
  Serial.println("BLE advertising (Cycling Power Service)...");
}

void configureWakeOnMotion() {
  // Configure LSM6DS3: low-power accel + wake on motion -> route to INT1
  // CTRL1_XL (0x10): 26Hz, ±2g recommended for low-power wake
  i2cWrite(LSM6DS3_ADDR, 0x10, 0x20);
  // WAKE_THS (0x5B): example value ~250 mg
  i2cWrite(LSM6DS3_ADDR, 0x5B, 0x04);
  // WAKE_DUR (0x5C): duration threshold
  i2cWrite(LSM6DS3_ADDR, 0x5C, 0x00);
  // TAP_CFG (0x58): enable wake-up event
  i2cWrite(LSM6DS3_ADDR, 0x58, 0xC0);
  // MD1_CFG (0x5E): route wake-up to INT1
  i2cWrite(LSM6DS3_ADDR, 0x5E, 0x20);
}

long averageCounts(int n, unsigned long timeoutMs) {
  long sum = 0;
  int k = 0;
  unsigned long t0 = millis();
  while (k < n && (millis() - t0) < timeoutMs) {
    if (scale.is_ready()) {
      sum += scale.read();
      k++;
    }
  }
  return (k > 0) ? (sum / k) : 0L;
}

void doTare() {
  Serial.println("Tare: ensure no load on pedal/crank.");
  delay(1000);
  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);
  Serial.print("Tare done. zeroOffsetCounts = ");
  Serial.println(zeroOffsetCounts);
}

void runCalibration(float knownMassKg) {
  Serial.println("=== Calibration ===");
  Serial.println("Step 1: ensure no load (will measure tare)...");
  doTare();

  Serial.print("Step 2: hang mass ");
  Serial.print(knownMassKg, 3);
  Serial.println(" kg at the pedal now and keep steady...");
  delay(2500);

  long loadedAvg = averageCounts(CAL_SAMPLES, 5000);
  long deltaCounts = loadedAvg - zeroOffsetCounts;

  const float g = 9.80665f;
  float forceN = knownMassKg * g; // force only (we keep scaleFactor in counts/N)

  if (forceN <= 0.0f) {
    Serial.println("Invalid mass. Calibration aborted.");
    return;
  }

  scaleFactor_counts_per_N = (float)deltaCounts / forceN; // counts per Newton

  Serial.print("deltaCounts = "); Serial.println(deltaCounts);
  Serial.print("forceN = "); Serial.print(forceN, 6); Serial.println(" N");
  Serial.print("scaleFactor_counts_per_N = ");
  Serial.println(scaleFactor_counts_per_N, 6);

  Serial.println("Calibration complete. scaleFactor is active now (no reflash).");
}

void sendCyclingPowerMeasurement(int16_t powerWatts) {
  // Flags: 16-bit (little-endian). Set Crank Revolution Data Present (bit 1).
  // See Cycling Power Measurement spec: Crank Revolution Data Present = bit 1.
  uint16_t flags = 0x0002; // bit1 = crank present

  uint8_t buf[8];
  int idx = 0;
  buf[idx++] = lowByte(flags);
  buf[idx++] = highByte(flags);

  // instantaneous power (sint16)
  buf[idx++] = lowByte((uint16_t)powerWatts);
  buf[idx++] = highByte((uint16_t)powerWatts);

  // cumulative crank revolutions (uint16) -- send lower 16 bits per spec
  uint16_t crc16 = (uint16_t)(cumulativeCrankRevs & 0xFFFF);
  buf[idx++] = lowByte(crc16);
  buf[idx++] = highByte(crc16);

  // last crank event time (uint16) in units of 1/1024s (already prepared elsewhere)
  buf[idx++] = lowByte(lastCrankEventTime);
  buf[idx++] = highByte(lastCrankEventTime);

  // write notification (will only be sent if client has subscribed)
  cpMeasurementChar.writeValue(buf, idx);
}

void processRevolution(float angVelRad) {
  // Compute average torque from accumulated HX711 samples
  float avgTorque = (torqueSampleCount > 0) ? (sumTorqueNm / (float)torqueSampleCount) : 0.0f;

  // Calculate instantaneous power (W)
  float powerF = avgTorque * angVelRad;
  if (powerF < 0.0f) powerF = 0.0f; // clamp negative to 0
  if (powerF > 32767.0f) powerF = 32767.0f;

  int16_t powerW = (int16_t)lroundf(powerF);

  // Send BLE notification (exactly once per revolution)
  sendCyclingPowerMeasurement(powerW);

  // Debug print (uncomment during bench testing)
  Serial.print("Rev ");
  Serial.print(cumulativeCrankRevs);
  Serial.print(" | power(W) = ");
  Serial.print(powerW);
  Serial.print(" | avgTorque(Nm) = ");
  Serial.print(avgTorque, 4);
  Serial.print(" | angVel(rad/s) = ");
  Serial.println(angVelRad, 3);

  // Reset accumulators for next revolution
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0;
}

void goToSleep() {
  // Power down HX711 and enter System OFF.
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, HIGH);
  delayMicroseconds(100);
  pinMode(HX_SCK, INPUT_PULLUP); // hold high via internal pullup while sleeping

  // MCU enters System OFF; will reset on wake via IMU INT1 event
  NRF_POWER->SYSTEMOFF = 1;
}
