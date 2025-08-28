#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "HX711.h"

// HX711 pins
#define HX_DT   2
#define HX_SCK  3

HX711 scale;

// IMU interrupt pin (LSM6DS3 INT1 wired to this pin)
#define MOTION_INT_PIN 7  

// Movement detection thresholds
const float MOVEMENT_THRESH_DPS = 2.0;     // deg/s threshold
const unsigned long IDLE_TIME_MS = 2000;   // must be idle this long to sleep

unsigned long lastMotionTime = 0;

// Forward declarations
void goToSleep();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Init IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Configure IMU interrupt pin
  pinMode(MOTION_INT_PIN, INPUT_PULLUP);   // IMU INT line -> MCU wake

  // Init HX711
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, LOW);   // ensure awake
  pinMode(HX_DT, INPUT);
  scale.begin(HX_DT, HX_SCK);

  delay(75); // allow HX711/bridge to settle after reset

  lastMotionTime = millis();
}

void loop() {
  // Read IMU gyro
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    float mag = sqrt(gx*gx + gy*gy + gz*gz);
    if (mag > MOVEMENT_THRESH_DPS) {
      lastMotionTime = millis();
    }
  }

  // HX711 sample
  if (scale.is_ready()) {
    long reading = scale.read_average(5);
    Serial.print("HX711 avg: ");
    Serial.println(reading);
  }

  // If idle too long → sleep
  if (millis() - lastMotionTime > IDLE_TIME_MS) {
    goToSleep();
  }

  delay(50);
}

// ---- Power down sequence ----
void goToSleep() {
  Serial.println("Going to sleep...");

  // 1. Put HX711 into power-down
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, HIGH);
  delayMicroseconds(100);  // >60 µs
  // Switch to INPUT_PULLUP so PD_SCK stays HIGH in System OFF
  pinMode(HX_SCK, INPUT_PULLUP);

  // 2. Configure IMU wake-on-motion
  // (Real implementation would configure LSM6DS3 registers here,
  // Arduino_LSM6DS3 wrapper doesn't expose wake-on-motion directly)

  // 3. Enter System OFF: lowest power, reset on wake
  NRF_POWER->SYSTEMOFF = 1;

  // <--- MCU and HX711 both sleeping now, will wake/reset on IMU INT --->
}
