---

# DIY Cycling Power Meter

A single-sided crank-based cycling power meter built around the **Seeed XIAO nRF52840 Sense**, a set of **strain gauges**, and the classic **HX711 load cell amplifier**. Cadence is measured using the onboard 6-axis IMU (gyro), and everything is transmitted over Bluetooth using the official **Cycling Power Profile (CPP)** so that bike computers and apps (Garmin, Wahoo, Zwift, etc.) recognize it as a real power meter.

---

## Features

* **Power measurement** via 4-gauge Wheatstone bridge + HX711.
* **Cadence detection** using the onboard LSM6DS3 gyro (no magnets).
* **Bluetooth Low Energy (BLE)** Cycling Power Profile (standardized).
* **Auto sleep / wake-on-motion**: goes into µA standby when idle, wakes when you touch the pedals.
* **Configurable inactivity timeout** (default 5s for testing, 60s recommended in use).
* Runs from a single **18650 Li-ion cell** for months.

---

## Why these components?

* **Seeed XIAO nRF52840 Sense**

  * Tiny, low-power, has BLE built in.
  * Includes a 6-axis IMU perfect for cadence detection.
  * Has built-in lithium cell charger (really slow but hey, its tiny)
* **HX711 load cell amplifier**

  * Cheap, easy to use, widely supported.
  * Commercial meters often use more advanced ADCs (TI ADS-series, custom ASICs), but HX711 works fine for DIY.
* **Strain gauges**

  * Standard foil gauges arranged in a full Wheatstone bridge give best sensitivity and thermal stability.
* **18650 battery**

  * High energy density, rechargeable, common in DIY.

---

## Hardware setup

* Mount 4 strain gauges on the **right crank arm** (single-sided measurement).
* Wire them as a **full Wheatstone bridge** → connect to HX711 breakout.
* HX711 `DOUT` → XIAO pin D2.
* HX711 `SCK`  → XIAO pin D3.
* IMU is onboard (LSM6DS3). Its INT1 can be connected to pin D7 if you want hardware wake.
* Power everything from the 3.3V regulated by the XIAO.

---

## Software overview

* **Cadence detection**:

  * Gyroscope outputs angular velocity in deg/s.
  * Smoothed with a simple low-pass filter.
  * Positive → negative → positive zero-crossing = one crank revolution.
* **Power calculation**:

  * HX711 reads torque via strain gauges.
  * Multiplied by cadence (angular velocity) to get Watts (calibration required).
* **BLE output**:

  * Implements Cycling Power Measurement characteristic (`0x2A63`).
  * Sends instantaneous power + cumulative crank revolutions + event time.
* **Low power**:

  * After `SLEEP_TIMEOUT_MS` of inactivity → HX711 powers down, MCU enters System OFF.
  * IMU stays in low-power motion detection (\~10 µA).
  * When crank moves → IMU wakes MCU → system restarts and advertises again.

---

## Calibration

* Mount crank securely.
* Hang a known weight at a fixed pedal length (e.g. 10 kg at 175 mm = 17.15 Nm torque).
* Read HX711 value → scale to match known torque.

---

## Limitations / notes

* Single-sided (right crank only) → doubles value to estimate total power.
* HX711 is not as precise as commercial solutions, but cheap and works well enough.
* Firmware currently uses a simple exponential moving average filter; better DSP could improve cadence stability.

---

## Getting started

1. Flash the Arduino sketch onto the XIAO nRF52840 Sense.
2. Connect all the modules together
3. Fit enclosure inside the crank
4. Pair with your bike computer or Zwift. It should show up as a standard **power + cadence sensor**.
5. Ride 🚴‍♂️.

---

## Roadmap / ideas

* Add left-right balance (second crank).
* Switch HX711 to ADS1232/ADS131M for higher resolution.
* Add ANT+ broadcasting.
* Smarter filtering for cadence detection.
* 3D-print a protective housing.

---
