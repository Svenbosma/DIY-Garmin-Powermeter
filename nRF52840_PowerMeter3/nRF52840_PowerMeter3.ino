/*
  DIY Powermeter v3.2
  - HX711 interrupt-driven torque sensor
  - BLE Cycling Power Service + UART calibration/tare
  - Cadence from BMI270/LSM6DS3: accel X zero-cross + gyro X sign
  - Wake-on-motion to exit SYSTEMOFF (using double tap)
*/

#include <bluefruit.h>
#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <LSM6DS3.h>
#include "nrf_gpio.h"
#include <nrf_sdm.h>


// ---------- config ----------
#define CRANK_LENGTH_M      0.1725f
#define ACCEL_THRESHOLD_G   0.05f
#define SLEEP_TIMEOUT_MS    60000UL
#define TARE_SAMPLES        200
#define CAL_SAMPLES         200
#define GYRO_FORWARD_THRESHOLD 0.0f
#define LOW_BATT_THRESHOLD 20
#define NUM_BATT_SAMPLES 10
#define BATTERY_UPDATE_INTERVAL 12000UL
#define BLINK_INTERVAL 300
#define NUM_BLINKS 3
#define FIFO_WATERMARK 30   // bytes (~5 gyro samples)

// ---------- pins ----------
#define HX_DT           2
#define HX_SCK          3
#define MOTION_INT_PIN  PIN_LSM6DS3TR_C_INT1

// Battery
unsigned long lastBattMs = 0;
unsigned long lastBlinkMs = 0;
uint8_t battPercent = 100;
uint8_t blinkCount = 0;
bool ledState = false;

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);
volatile bool fifoIRQ = false;
void imuISR() { fifoIRQ = true; }

// Torque + IMU accumulators
float sumTorqueNm = 0.0f;
int torqueSampleCount = 0;
int imuSampleCount = 0;
float sumGyroZ = 0.0f;
unsigned long lastRevMs = 0;
uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0;
bool prevAccelPositive = false;
volatile bool imuFifoReady = false;

// HX711
HX711 scale;
long zeroOffsetCounts = 0;
float scaleFactor_counts_per_N = NAN;
volatile bool hx711ReadyFlag = false;

// prototypes
void hx711ISR();
void imuISR();
void processIMUFIFO();
void configureIMUSettings();
void setupIMUFIFO();
void setupBLE();
void doTare();
void runCalibration(float knownMassKg);
void sendCyclingPowerMeasurement(int16_t powerWatts,
                                 uint16_t cumulativeCrankRevs,
                                 uint16_t lastCrankEventTime);
void goToSystemOff();
void batteryCheckAndLED();
void CadencePowerCalc(unsigned long revolutionMs);
float ReadGxFIFO();

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
  // -- Peripheral and pin setup --
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {}
  logPrintln("DIY Powermeter v3.1 start");
  Wire.begin();

  // --- Battery pin setup ---
  analogReadResolution(12); 
  analogReference(AR_INTERNAL_2_4);
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
  pinMode(VBAT_ENABLE, OUTPUT); 
  digitalWrite(VBAT_ENABLE, LOW);
  pinMode(PIN_VBAT, INPUT);

  // ---- HX711 setup ----
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP);
  scale.begin(HX_DT, HX_SCK);
  delay(75);
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

  delay(1000);
  logPrintln("Starting IMU...");
  // Apply all sensor/FIFO settings before begin(), matching the working FIFO test.
  configureIMUSettings();
  int imuStatus = myIMU.begin();
  logPrint("IMU begin result = ");
  logPrintln(String(imuStatus));

  if (imuStatus != 0) {
    logPrintln("IMU init failed!");
    while (1);
  }
  setupIMUFIFO();

  // ---- Startup routine ----
  setupBLE();
  lastRevMs = millis();
  logPrintln("Ready. Commands via UART: 'c'=calib, 't'=tare, 'm <kg>'=custom calib.");
}

void loop() {
  // HX711
  if (hx711ReadyFlag) {
    hx711ReadyFlag = false;
    long raw = scale.read();
    long net = raw - zeroOffsetCounts;

    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N;
      float torqueNm = forceN * CRANK_LENGTH_M;
      accumulateTorque(torqueNm);
    }
  }
  
  // IMU
  if (fifoIRQ) {
    fifoIRQ = false;
    drainFifoAvgGx();
  }

  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    logPrintln("No pedaling -> deep sleep");
    goToSystemOff();
  }

  batteryCheckAndLED();
  sd_app_evt_wait();
}
