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

// ---------- config ----------
#define CRANK_LENGTH_M      0.1725f
#define ACCEL_THRESHOLD_G   0.05f
#define SLEEP_TIMEOUT_MS    60000UL
#define TARE_SAMPLES        200
#define CAL_SAMPLES         200
#define GYRO_FORWARD_THRESHOLD 0.0f
#define LOW_BATT_THRESHOLD 20
#define NUM_BATT_SAMPLES 10
#define BATTERY_UPDATE_INTERVAL 6000UL
#define BLINK_INTERVAL 300
#define NUM_BLINKS 3

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

// Torque + IMU accumulators
float sumTorqueNm = 0.0f;
int torqueSampleCount = 0;
int imuSampleCount = 0;
float sumGyroX = 0.0f, sumAccelX = 0.0f;
unsigned long lastRevMs = 0;
uint32_t cumulativeCrankRevs = 0;
uint16_t lastCrankEventTime = 0;
bool prevAccelPositive = false;

// HX711
HX711 scale;
long zeroOffsetCounts = 0;
float scaleFactor_counts_per_N = NAN;
volatile bool hx711ReadyFlag = false;

// prototypes
void hx711ISR();
void setupBLE();
void doTare();
void runCalibration(float knownMassKg);
void sendCyclingPowerMeasurement(int16_t powerWatts);
void handleUART();
void goToSystemOff();
void batteryCheckAndLED();
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

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {}
  logPrintln("DIY Powermeter v3.1 start");

  Wire.begin();
  analogReadResolution(12); 

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

void loop() {
  handleUART();  // BLE UART handling

  // HX711
  if (hx711ReadyFlag || scale.is_ready()) {
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
  float gx = myIMU.readFloatGyroX();
  float ax = myIMU.readFloatAccelX();
  accumulateIMU(gx, ax);

  if (detectRevolution(gx, ax)) {
    float avgTorque = (torqueSampleCount > 0) ? (sumTorqueNm / torqueSampleCount) : 0.0f;
    float avgGyro = (imuSampleCount > 0) ? (sumGyroX / imuSampleCount) : 0.0f;

    unsigned long now = millis();
    float dt = (now - lastRevMs) / 1000.0f;
    lastRevMs = now;
    if (dt < 0.2f) dt = 0.2f;
    float cadence_rpm = 60.0f / dt;

    float angVelRad = avgGyro * DEG_TO_RAD;
    float powerW = calcPower(avgTorque, angVelRad);

    cumulativeCrankRevs++;
    lastCrankEventTime = (uint16_t)((now * 1024UL) / 1000UL);

    sendCyclingPowerMeasurement((int16_t)powerW);

    sumTorqueNm = 0.0f; torqueSampleCount = 0;
    sumGyroX = 0.0f; sumAccelX = 0.0f; imuSampleCount = 0;

    logPrint("Cadence="); logPrint(cadence_rpm,1);
    logPrint("rpm  Power="); logPrintln(powerW,1);
  }

  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    logPrintln("No pedaling -> deep sleep");
    goToSystemOff();
  }

  batteryCheckAndLED();
}
