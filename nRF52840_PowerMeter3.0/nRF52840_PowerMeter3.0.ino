/*
  HX711 interrupt-driven, idle between samples, per-rev BLE CPP

  - HX711 DOUT -> interrupt (FALLING) signals new sample available
  - ISR sets flag, wakes CPU; main loop reads sample once per event
  - Between HX711 events the MCU executes __WFE() (low-power wait-for-event)
  - One BLE Cycling Power Measurement sent per crank revolution
  - Calibration/tare helpers via Serial: 'c' (10kg), 't' (tare), 'm <kg>' custom
*/

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "HX711.h"

// ---------- pins ----------
#define HX_DT           2    // HX711 DOUT (data ready -> low)
#define HX_SCK          3
#define MOTION_INT_PIN  7    // IMU INT1 -> XIAO (used only for wake-from-deep-sleep)

// ---------- config ----------
#define CRANK_LENGTH_M          0.1725f
#define CADENCE_THRESHOLD_DPS   30.0f
#define GYRO_FILTER_ALPHA       0.15f
#define SLEEP_TIMEOUT_MS        60000UL
#define TARE_SAMPLES            200
#define CAL_SAMPLES             200

// ---------- globals ----------
HX711 scale;
long zeroOffsetCounts = 0;
float scaleFactor_counts_per_N = NAN; // counts per Newton after calibration

uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0; // 1/1024s units

// cadence detection
volatile bool hx711ReadyFlag = false; // set in ISR when HX711 DOUT falls
float lastFilteredGz = 0.0f;
bool prevPositive = false;
unsigned long lastRevMs = 0;

// per-rev accumulation
float sumTorqueNm = 0.0f;
uint16_t torqueSampleCount = 0;

// BLE objects
BLEService cyclingPowerService("1818");
BLECharacteristic cpMeasurementChar("2A63", BLENotify, 8);
BLEUnsignedLongCharacteristic cpFeatureChar("2A65", BLERead);

#define LSM6DS3_ADDR 0x6A

// ---------- prototypes ----------
void hx711ISR();
void setupBLE();
void configureWakeOnMotion();
long averageCounts(int n, unsigned long timeoutMs);
void doTare();
void runCalibration(float knownMassKg);
void processRevolution(float angVelRad);
void sendCyclingPowerMeasurement(int16_t powerWatts);
void goToDeepSleep();

// ---------- implementation ----------

void hx711ISR() {
  // Simple ISR: mark new sample ready. Keep it extremely short.
  hx711ReadyFlag = true;
  // ARM WFE/wake will be triggered automatically by the interrupt.
}

static inline void i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // HX711 init
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP); // DOUT is open-drain / pulldown when ready; pullup helps idle high
  scale.begin(HX_DT, HX_SCK);
  delay(75);

  // attach interrupt on HX711 DOUT falling edge (data ready)
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

  // IMU init
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  pinMode(MOTION_INT_PIN, INPUT); // IMU INT1 push-pull

  configureWakeOnMotion();

  // BLE
  setupBLE();

  // Tare once at boot
  doTare();

  lastRevMs = millis();

  Serial.println("Ready. Commands: 'c' calibrate(10kg), 't' tare, 'm <kg>' custom calib.");
}

/* ----------------------
   Main loop is intentionally tiny:
    - handle serial cmds (non-blocking)
    - idle-wait for HX711 data ready (using __WFE)
    - when event: read sample, read gyro, accumulate, check revolution
   ---------------------- */
void loop() {
  BLE.poll(); // process BLE stack events, callbacks

  // Serial commands (non-blocking)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("c")) {
      runCalibration(10.0f);
    } else if (cmd.equalsIgnoreCase("t")) {
      doTare();
    } else if (cmd.startsWith("m")) {
      float kg = cmd.substring(1).toFloat();
      if (kg > 0.0f) runCalibration(kg);
      else Serial.println("Usage: m <mass_kg>");
    }
  }

  // Idle wait for HX711 event (light sleep). Wakes on any interrupt (HX711 DOUT, Serial, BLE, etc.)
  // Clear flag before sleeping to avoid race where ISR already occurred.
  hx711ReadyFlag = false;

  // If already have a ready flag (rare) skip entering WFE.
  if (!hx711ReadyFlag) {
    // Ensure any previous events don't get lost
    __SEV();    // set event to ensure WFE won't hang if an event is pending
    __WFE();    // wait for event (light sleep)
    // After WFE returns, some interrupt happened.
  }

  // If hx711ReadyFlag set by ISR, handle the event
  if (hx711ReadyFlag || scale.is_ready()) {
    // Read HX711 (non-blocking now because data ready)
    long raw = scale.read(); // 24-bit signed value (blocking while clocking, but fast)
    long net = raw - zeroOffsetCounts;

    // convert to torque if calibrated
    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N;
      float torqueNm = forceN * CRANK_LENGTH_M;
      sumTorqueNm += torqueNm;
      torqueSampleCount++;
    }

    // Read gyro once (we read gyro at HX711 sample rate)
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz); // deg/s
      // filter Z axis
      float filtered = GYRO_FILTER_ALPHA * gz + (1.0f - GYRO_FILTER_ALPHA) * lastFilteredGz;
      lastFilteredGz = filtered;

      bool isPositive = (filtered > +CADENCE_THRESHOLD_DPS);
      bool isNegative = (filtered < -CADENCE_THRESHOLD_DPS);

      if (prevPositive && isNegative) {
        prevPositive = false;
      } else if (!prevPositive && isPositive) {
        prevPositive = true;

        // revolution finished -> compute angular velocity from lastRevMs
        unsigned long now = millis();
        float dt = (now - lastRevMs) / 1000.0f;
        float angVelRad = (dt > 0.0f) ? (2.0f * PI / dt) : 0.0f;

        // update counters
        cumulativeCrankRevs++;
        lastCrankEventTime = (uint16_t)((now * 1024UL) / 1000UL);

        // compute & send BLE
        processRevolution(angVelRad);

        lastRevMs = now;
      }
    }
  }

  // Auto deep-sleep after long inactivity (we still do deep sleep for storage)
  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    Serial.println("No pedaling -> deep sleep");
    delay(50);
    goToDeepSleep();
  }
}

/* ---------------- supporting functions ---------------- */

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE begin failed");
    while (1);
  }
  BLE.setLocalName("DIY-Powermeter");
  BLE.setAdvertisedService(cyclingPowerService);

  cyclingPowerService.addCharacteristic(cpMeasurementChar);
  cyclingPowerService.addCharacteristic(cpFeatureChar);
  BLE.addService(cyclingPowerService);

  cpFeatureChar.writeValue(0); // no extra features
  BLE.advertise();
  Serial.println("BLE advertising (Cycling Power Service)");
}

void configureWakeOnMotion() {
  // LSM6DS3: low-power accel + wake-on-motion -> INT1
  i2cWrite(LSM6DS3_ADDR, 0x10, 0x20); // CTRL1_XL: 26Hz ±2g
  i2cWrite(LSM6DS3_ADDR, 0x5B, 0x04); // WAKE_THS ~250mg
  i2cWrite(LSM6DS3_ADDR, 0x5C, 0x00); // WAKE_DUR 0
  i2cWrite(LSM6DS3_ADDR, 0x58, 0xC0); // TAP_CFG enable
  i2cWrite(LSM6DS3_ADDR, 0x5E, 0x20); // route wake->INT1
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
  Serial.println("Tare: remove load from pedal");
  delay(1000);
  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);
  Serial.print("zeroOffsetCounts = "); Serial.println(zeroOffsetCounts);
}

void runCalibration(float knownMassKg) {
  Serial.println("Calibration start");
  doTare();
  Serial.print("Hang "); Serial.print(knownMassKg); Serial.println(" kg on pedal now");
  delay(2500);
  long loadedAvg = averageCounts(CAL_SAMPLES, 5000);
  long delta = loadedAvg - zeroOffsetCounts;
  const float g = 9.80665f;
  float forceN = knownMassKg * g;
  if (forceN <= 0.0f) {
    Serial.println("Bad mass; abort");
    return;
  }
  scaleFactor_counts_per_N = (float)delta / forceN;
  Serial.print("deltaCounts="); Serial.println(delta);
  Serial.print("forceN="); Serial.println(forceN, 6);
  Serial.print("scaleFactor_counts_per_N="); Serial.println(scaleFactor_counts_per_N, 6);
  Serial.println("Calibration done (active immediately)");
}

void processRevolution(float angVelRad) {
  float avgTorque = (torqueSampleCount > 0) ? (sumTorqueNm / (float)torqueSampleCount) : 0.0f;
  // compute power
  float pwr = avgTorque * angVelRad;
  if (pwr < 0.0f) pwr = 0.0f;
  if (pwr > 32767.0f) pwr = 32767.0f;
  int16_t powerW = (int16_t)lroundf(pwr);

  // send one BLE packet per revolution
  sendCyclingPowerMeasurement(powerW);

  // debug
  Serial.print("Rev "); Serial.print(cumulativeCrankRevs);
  Serial.print(" | P(W)="); Serial.print(powerW);
  Serial.print(" | avgTorque="); Serial.print(avgTorque, 4);
  Serial.print(" | angVel="); Serial.println(angVelRad, 3);

  // reset accumulators
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0;
}

void sendCyclingPowerMeasurement(int16_t powerWatts) {
  // Flags: set bit1 = Crank Revolution Data Present (0x0002)
  uint16_t flags = 0x0002;

  uint8_t buf[8];
  int idx = 0;
  buf[idx++] = lowByte(flags);
  buf[idx++] = highByte(flags);

  // instantaneous power (sint16)
  buf[idx++] = lowByte((uint16_t)powerWatts);
  buf[idx++] = highByte((uint16_t)powerWatts);

  // cumulative crank revs (uint16, lower 16 bits)
  uint16_t revs16 = (uint16_t)(cumulativeCrankRevs & 0xFFFF);
  buf[idx++] = lowByte(revs16);
  buf[idx++] = highByte(revs16);

  // last crank event time (uint16, 1/1024s)
  buf[idx++] = lowByte(lastCrankEventTime);
  buf[idx++] = highByte(lastCrankEventTime);

  cpMeasurementChar.writeValue(buf, idx);
}

void goToDeepSleep() {
  // power down HX711 and deep sleep (System OFF)
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, HIGH);
  delayMicroseconds(100);
  pinMode(HX_SCK, INPUT_PULLUP);

  NRF_POWER->SYSTEMOFF = 1;
}
