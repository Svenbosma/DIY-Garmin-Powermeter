/*
  DIY Powermeter v3.2
  - HX711 interrupt-driven torque sensor
  - BLE Cycling Power Service + UART calibration/tare
  - Cadence from LSM6DS3 gyro integration
  - Wake-on-motion to exit SYSTEMOFF
 */

#include <bluefruit.h>
#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <LSM6DS3.h>
#include "nrf_gpio.h"
#include <nrf_sdm.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#define CAL_FILE   "/calibration.txt"
#define TARE_FILE  "/tare.txt"

File calFile(InternalFS);
File tareFile(InternalFS);

// ---------- config ----------
#define CRANK_LENGTH_M      0.1725f
#define SLEEP_TIMEOUT_MS    60000UL
#define TARE_SAMPLES        200
#define CAL_SAMPLES         200
#define NUM_BATT_SAMPLES 10
#define BATTERY_UPDATE_INTERVAL 12000UL

// ---------- pins ----------
#define HX_DT           2
#define HX_SCK          3
#define MOTION_INT_PIN  PIN_LSM6DS3TR_C_INT1

// Battery state
unsigned long lastBattMs = 0;
uint8_t battPercent = 100;

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Measurement state
float sumTorqueNm = 0.0f;
int torqueSampleCount = 0;
unsigned long lastRevMs = 0;
uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0;
unsigned long revolutionTimestamp = 0;
volatile bool calibrationActive = false;

// HX711
HX711 scale;
long zeroOffsetCounts = 0;
float scaleFactor_counts_per_N = NAN;
volatile bool hx711ReadyFlag = false;

// Function prototypes
void hx711ISR();
float integrateYZ(float gyroY, float gyroZ);
bool detectRevolution(unsigned long sampleMillis, float degZ);
void accumulateTorque(float torqueNm);
void setupBLE();
void doTare();
void runCalibration(float knownMassKg);
void sendCyclingPowerMeasurement(int16_t powerWatts,
                                 uint16_t cumulativeCrankRevs,
                                 uint16_t lastCrankEventTime);
void goToSystemOff();
void batteryCheckAndLED();
void CadencePowerCalc(unsigned long revolutionMs);
void loadFlashValues();
void saveCalibration();
void saveTare();
void uartRXWriteCallback(uint16_t conn_hdl,
                         BLECharacteristic* chr,
                         uint8_t* data,
                         uint16_t len);
void processUARTCommand(String cmd);
void setupWakeUpInterrupt();

void logPrint(const String &msg);
void logPrintln(const String &msg);
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

void cpControlPointWriteCallback(uint16_t conn_hdl,
                                 BLECharacteristic* chr,
                                 uint8_t* data,
                                 uint16_t len);

void setup() {
  // Bring up serial logging first so startup diagnostics are visible.
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {}
  logPrintln("DIY Powermeter v3.2 start");
  Wire.begin();

  // Battery measurement and charge-current pin setup for the XIAO board.
  analogReadResolution(12); 
  analogReference(AR_INTERNAL_2_4);
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
  pinMode(VBAT_ENABLE, OUTPUT); 
  digitalWrite(VBAT_ENABLE, LOW);
  pinMode(PIN_VBAT, INPUT);

  // HX711 runs from the data-ready interrupt; keep SCK low when active.
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP);
  scale.begin(HX_DT, HX_SCK);
  delay(75);
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

  delay(1000);
  logPrintln("Starting IMU...");
  int imuStatus = myIMU.begin();
  logPrint("IMU begin result = ");
  logPrintln(String(imuStatus));

  if (imuStatus != 0) {
    logPrintln("IMU init failed!");
    while (1);
  }

  // Load persisted tare/calibration before starting BLE service.
  loadFlashValues();
  setupBLE();
  lastRevMs = millis();
  logPrintln("Ready. Commands via UART: 'c'=calib, 't'=tare, 'm <kg>'=custom calib.");
}

void loop() {
  // Each HX711 sample updates torque and provides the pacing for the main loop.
  if (hx711ReadyFlag) {
    hx711ReadyFlag = false;
    long raw = scale.read();
    long net = raw - zeroOffsetCounts;

    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N;
      float torqueNm = forceN * CRANK_LENGTH_M;
      accumulateTorque(torqueNm);
    }

    float integratedYz = integrateYZ(myIMU.readFloatGyroY(), myIMU.readFloatGyroZ());
    if (detectRevolution(millis(), integratedYz)) {
      CadencePowerCalc(revolutionTimestamp);
    }
  }

  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS && !calibrationActive) {
    logPrintln("No pedaling -> deep sleep");
    goToSystemOff();
  }

  batteryCheckAndLED();
  sd_app_evt_wait();
}
