/*
  DIY Powermeter v3.2
  - HX711 interrupt-driven torque sensor
  - BLE Cycling Power Service + UART calibration/tare
  - Cadence from BMI270/LSM6DS3: accel X zero-cross + gyro X sign
  - Wake-on-motion to exit SYSTEMOFF (using double tap)
*/

#include <ArduinoBLE.h>
//#include <Adafruit_TinyUSB.h>
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
void CadencePowerCalc();
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

void setup() {
  // -- Peripheral and pin setup --
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {}
  logPrintln("DIY Powermeter v3.1 start");
  Wire.begin();
  analogReadResolution(12); 

  // ---- HX711 setup ----
  nrf_gpio_cfg_output(13);   // P0.13
  nrf_gpio_pin_clear(13);    // LOW = 100 mA
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP);
  scale.begin(HX_DT, HX_SCK);
  delay(75);
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

/*  // ---- Imu setup ----
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = 2000;       // safe for up to ~1200 °/s
  myIMU.settings.gyroSampleRate = 833;   // Hz
  myIMU.settings.gyroBandWidth = 100;    // Hz low-pass filter
  myIMU.settings.gyroFifoEnabled = 1;
  myIMU.settings.gyroFifoDecimation = 1;
  // Disable accel & temp & timestamp
  myIMU.settings.accelEnabled = 0;
  myIMU.settings.tempEnabled  = 0;
  myIMU.settings.timestampEnabled = 0;
  myIMU.settings.timestampFifoEnabled = 0;
  //Non-basic mode settings
  myIMU.settings.commMode = 1;
  // FIFO settings
  myIMU.settings.fifoThreshold = 0;     // not   used
  myIMU.settings.fifoSampleRate = 800;  // match gyro ODR
  myIMU.settings.fifoModeWord = 6;      // continuous mode*/
  if (myIMU.begin() != 0) {
    logPrintln("IMU init failed!");
    while(1);
  }
  //myIMU.fifoBegin();
  //myIMU.fifoClear();

  // ---- Startup routine ----
  setupBLE();
  doTare();
  lastRevMs = millis();
  logPrintln("Ready. Commands via UART: 'c'=calib, 't'=tare, 'm <kg>'=custom calib.");
}

void loop() {
  float hz = getAverageHz();
  // Only print once per second when updated
  static float lastHz = 0.0;
  if (hz != lastHz) {
    logPrint("Average Hz: ");
    logPrintln(hz, 2);  // Two decimal precision
    lastHz = hz;
  }

  handleUART();  // BLE UART handling

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
  //  gx = ReadGxFIFO();
  //logPrint(gx);
  //logPrint(" Sampled vs normal ");
  float gx = myIMU.readFloatGyroX();
  //logPrintln(gx);
  accumulateIMU(gx);

  if (detectRevolution(gx)) {
    CadencePowerCalc();
  }

  if ((millis() - lastRevMs) > SLEEP_TIMEOUT_MS) {
    logPrintln("No pedaling -> deep sleep");
    goToSystemOff();
  }

  batteryCheckAndLED();
}

// --- Global variables ---
unsigned long lastTime = 0;
unsigned long loopCount = 0;
float avgHz = 0.0;

float getAverageHz() {
  loopCount++;  // Count each loop iteration

  unsigned long now = millis();
  unsigned long elapsed = now - lastTime;

  if (elapsed >= 1000) {  // Every 1 second
    avgHz = (float)loopCount * 1000.0 / (float)elapsed;  // Loops per second
    loopCount = 0;
    lastTime = now;
  }

  return avgHz;
}