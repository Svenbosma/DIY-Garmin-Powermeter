#include <Arduino.h>
#include <ArduinoBLE.h>
#include "HX711.h"

// ====== IMU ======
#include "Arduino_LSM6DS3.h"   // For XIAO BLE Sense this matches LSM6DS3TR-C
// If using a different IMU lib, adapt: begin(), gyroscopeAvailable(), readGyroscope()

// ====== Optional BLE Calibration (Control Point) ======
// Comment this line to remove BLE calibration support
#define ENABLE_CALIBRATION

// ====== HX711 pins (good, conflict-free choices on XIAO) ======
#define HX711_DOUT 2
#define HX711_SCK  3

// ====== Globals / Parameters ======
HX711 scale;

// Set after span calibration (static weight test):
// raw_counts / scaleFactor => Newtons at the pedal (see 'forceN' calc below).
// Compute this from your weight-hang torque test and update permanently.
volatile float scaleFactor = 1000.0f;      // *** PLACEHOLDER *** set after your slope calibration

// Crank geometry
const float crankLength_m = 0.1725f;       // 172.5 mm crank

// BLE Cycling Power Service (0x1818)
BLEService cyclingPowerService("1818");

// Measurement (0x2A63, notify)
BLECharacteristic cpMeasurementChar("2A63", BLENotify, 20);

// Features (0x2A65, read)
BLECharacteristic cpFeatureChar("2A65", BLERead, 4);

// Sensor Location (0x2A5D, read) 13 = Right Crank per Assigned Numbers
BLECharacteristic cpLocationChar("2A5D", BLERead, 1);

// Optional Control Point (0x2A66, write+indicate)
#ifdef ENABLE_CALIBRATION
BLECharacteristic cpControlPointChar("2A66", BLEWrite | BLEIndicate, 20);
#endif

// ====== Cadence state (spec wants cumulative revs + last event time only) ======
static uint16_t cumulativeCrankRevs = 0;
static uint16_t lastCrankEventTime1024 = 0;   // 1/1024 s units

// Angle integration and event detection
static float crankAngle = 0.0f;              // [rad], wraps 0..2π
static unsigned long lastUpdate_us = 0;

// Gyro filtering for power stability
static float angVelFiltered = 0.0f;          // [rad/s]
const float angVelAlpha = 0.12f;             // EMA factor (tune 0.05..0.2)

// Torque/power
static float torqueNm = 0.0f;
static float powerW = 0.0f;

// Simple zero-offset (tare) stored in RAM (you can persist to flash/EEPROM)
static long zeroOffsetCounts = 0;

// ====== Helpers ======
static inline uint16_t msTo1024(uint32_t ms) {
  // spec wants units of 1/1024 s, 16-bit with wrap
  // (ms * 1024 / 1000) modulo 65536
  return (uint16_t)((ms * 1024UL) / 1000UL);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // HX711
  scale.begin(HX711_DOUT, HX711_SCK);
  delay(50);
  scale.set_gain(128);           // default channel A gain
  scale.tare();                  // internal HX711 tare to start
  zeroOffsetCounts = 0;          // our own logical zero for torque

  // IMU
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1) { delay(1000); }
  }
  // Optionally set gyro range if your library exposes it:
  // IMU.setGyroRange(1000); // ±1000 °/s handles up to ~166 RPM comfortably

  // BLE
  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1) { delay(1000); }
  }

  // Add CPS characteristics
  cyclingPowerService.addCharacteristic(cpMeasurementChar);
  cyclingPowerService.addCharacteristic(cpFeatureChar);
  cyclingPowerService.addCharacteristic(cpLocationChar);

#ifdef ENABLE_CALIBRATION
  cyclingPowerService.addCharacteristic(cpControlPointChar);
#endif

  BLE.addService(cyclingPowerService);

  // Features bitfield: declare support for crank torque & crank rev data
  // (bit assignments per CPS spec; conservative minimal set)
  // 0x00000010 = Crank Torque (Torque-based power)
  // 0x00000008 = Crank Revolution Data supported
  uint32_t feature = 0x00000010 | 0x00000008;
  cpFeatureChar.writeValue((uint8_t*)&feature, 4);

  // Sensor Location: Right Crank = 13
  const uint8_t rightCrank = 13;
  cpLocationChar.writeValue(&rightCrank, 1);

#ifdef ENABLE_CALIBRATION
  // Handle writes to Control Point
  cpControlPointChar.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic ch) {
    uint8_t req[20] = {0};
    int n = ch.valueLength();
    if (n <= 0) return;
    ch.readValue(req, n);

    const uint8_t opCode = req[0];
    // CPS OpCode 0x03: Start Offset Compensation (zero/tare)
    if (opCode == 0x03) {
      // Take multiple HX711 reads for stable zero
      long avg = scale.read_average(25);
      zeroOffsetCounts = avg;  // store as our logical zero

      // Build Response: 0x20 (Response Code), 0x03 (request op), 0x01 (Success)
      // Optionally include a 2-byte offset echo (implementation-defined)
      uint8_t rsp[5];
      rsp[0] = 0x20;     // Response Code
      rsp[1] = 0x03;     // Request Op Code echoed
      rsp[2] = 0x01;     // Success
      int16_t echo = (int16_t)(0); // sending 0 as we absorbed offset internally
      rsp[3] = (uint8_t)(echo & 0xFF);
      rsp[4] = (uint8_t)((echo >> 8) & 0xFF);

      ch.writeValue(rsp, 5);   // set value before indicate
      ch.indicate();           // send indication per spec
      Serial.println("CPS: Offset compensation (zero) done.");
    } else {
      // Unsupported Op Code → standard Response Code with 'Op Code Not Supported' (0x02)
      uint8_t rsp[3];
      rsp[0] = 0x20;  // Response Code
      rsp[1] = opCode;
      rsp[2] = 0x02;  // Op Code not supported
      ch.writeValue(rsp, 3);
      ch.indicate();
    }
  });
#endif

  BLE.setDeviceName("DIY Right-Crank PM");
  BLE.setLocalName("DIY Right-Crank PM");
  BLE.advertise();

  lastUpdate_us = micros();
  Serial.println("CPS ready: notifying on each crank revolution.");
}

// ====== Loop ======
void loop() {
  BLE.poll();
  updateIMUAndMaybeNotify();
}

// ====== Core sensing & event logic ======
void updateIMUAndMaybeNotify() {
  // We only care about gyro; accelerometer could be added for long-term drift correction
  float gx, gy, gz;
  if (!IMU.gyroscopeAvailable()) return;

  IMU.readGyroscope(gx, gy, gz);         // °/s
  unsigned long now_us = micros();
  float dt = (now_us - lastUpdate_us) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) {         // guard against long pauses
    lastUpdate_us = now_us;
    return;
  }
  lastUpdate_us = now_us;

  // Choose axis aligned with crank rotation; adapt if needed:
  // here we assume +Z increases when pedaling forward.
  float angVel_rad_s = gz * DEG_TO_RAD;

  // Filter angular velocity to stabilize power
  angVelFiltered = angVelAlpha * angVel_rad_s + (1.0f - angVelAlpha) * angVelFiltered;

  // Integrate UNFILTERED rate to maintain correct angle progression (less bias from filter phase lag)
  crankAngle += angVel_rad_s * dt;

  // Wrap to [0, 2π)
  if (crankAngle >= TWO_PI) crankAngle -= TWO_PI;
  else if (crankAngle < 0.0f) crankAngle += TWO_PI;

  // Zero-crossing with hysteresis to avoid double-counting
  // We detect the transition from >π to <θ_low (~0 rad)
  static bool halfPassed = false;
  const float thetaLow = 0.08f;          // ~4.6°
  if (!halfPassed && crankAngle > PI) halfPassed = true;

  if (halfPassed && crankAngle < thetaLow) {
    // ======= CRANK REVOLUTION EVENT =======
    halfPassed = false;

    // Timestamp in 1/1024s units (16-bit wrap, per spec)
    lastCrankEventTime1024 = msTo1024(millis());
    cumulativeCrankRevs++;

    // --- Update torque from HX711 ---
    // Take a small moving average to reduce noise (5 samples @ ~80Hz HX711 ~ OK)
    long raw = scale.read_average(5);

    // Remove our zero offset (set by CP calibration or initial tare)
    long netCounts = raw - zeroOffsetCounts;

    // Convert counts to Newtons at the pedal (you define scaleFactor via static weight test)
    // Example: scaleFactor [counts per Newton]  =>  N = netCounts / scaleFactor
    // If you prefer to define scaleFactor as [counts per N], keep division as written.
    float forceN = (scaleFactor != 0.0f) ? ( (float)netCounts / scaleFactor ) : 0.0f;

    // Torque = F * crank length (assuming pedal force vector approx. tangential near max torque)
    torqueNm = forceN * crankLength_m;

    // Instantaneous power at event time using FILTERED angular velocity
    powerW = torqueNm * angVelFiltered;

    // Send CPS notification
    sendCyclingPowerMeasurement();
  }
}

// ====== BLE packet composer ======
void sendCyclingPowerMeasurement() {
  // Flags (little endian):
  // bit 5 (0x20): Crank Revolution Data Present
  // (no pedal balance, no wheel data, no vector, etc.)
  uint16_t flags = 0x20;

  uint8_t buf[20] = {0};
  buf[0] = (uint8_t)(flags & 0xFF);
  buf[1] = (uint8_t)((flags >> 8) & 0xFF);

  // Instantaneous Power (signed, in watts)
  int16_t instPower = (int16_t)roundf(powerW);
  buf[2] = (uint8_t)(instPower & 0xFF);
  buf[3] = (uint8_t)((instPower >> 8) & 0xFF);

  // Crank Rev Data (CumulativeCrankRevs uint16 + LastCrankEventTime uint16)
  buf[4] = (uint8_t)(cumulativeCrankRevs & 0xFF);
  buf[5] = (uint8_t)((cumulativeCrankRevs >> 8) & 0xFF);

  buf[6] = (uint8_t)(lastCrankEventTime1024 & 0xFF);
  buf[7] = (uint8_t)((lastCrankEventTime1024 >> 8) & 0xFF);

  cpMeasurementChar.writeValue(buf, 8);

  // Debug
  Serial.print("Rev# "); Serial.print(cumulativeCrankRevs);
  Serial.print("  P="); Serial.print(instPower);
  Serial.print(" W  τ="); Serial.print(torqueNm, 3);
  Serial.print(" Nm  ω_f="); Serial.print(angVelFiltered, 2);
  Serial.println(" rad/s");
}
