# DIY Cycling Power Meter

A **single-sided crank-based cycling power meter** built around the **Seeed XIAO nRF52840 Sense**, **BF1K-3AA foil strain gauges**, and the classic **HX711 load cell amplifier**.

Cadence is measured using the onboard **6-axis IMU (gyro)** and transmitted via **Bluetooth Low Energy using the Cycling Power Profile (CPP)** so cycling computers and apps (Garmin, Wahoo, Zwift, TrainerRoad, etc.) recognize it as a standard power meter.

The system is mounted on the **left crank arm**, measuring torque applied to the crank and estimating total rider power from the single-sided measurement.

---

# Mounted on the Bike

| | |
|---|---|
| Mounted on left crank | <img src="images/bike_left_crank_mount.jpg" width="400px"> |
| Electronics enclosure | <img src="images/bike_electronics_mount.jpg" width="400px"> |
| Close-up of strain gauges | <img src="images/crank_strain_gauges_installed.jpg" width="400px"> |

Four **BF1K-3AA strain gauges** are bonded to the crank to measure deformation under pedaling load.  
The signal is amplified using an **HX711** and processed by the **XIAO nRF52840**.

The entire unit runs from a **300 mAh LiPo battery**, providing long runtime while keeping the system compact and lightweight.

---

# Hardware Overview

| Component | Image |
|-----------|-------|
| Seeed XIAO nRF52840 Sense | <img src="images/xiao.png" width="180px"> |
| HX711 Load Cell Amplifier | <img src="images/hx711.png" width="180px"> |
| BF1K-3AA Strain Gauges | <img src="images/strain_gauges.png" width="180px"> |
| Left Crank Arm | <img src="images/crank.png" width="180px"> |
| 300 mAh LiPo Battery | <img src="images/lipo_300mah.png" width="180px"> |
| Weatherproof Magnetic Charger | <img src="images/connector.png" width="180px"> |

---

# Features

- **Power measurement** using a 4-gauge Wheatstone bridge and HX711 amplifier  
- **Cadence detection** using the onboard LSM6DS3 gyro (no magnets required)  
- **Bluetooth Low Energy (BLE)** Cycling Power Profile compatibility  
- Works with **Garmin, Wahoo, Zwift, TrainerRoad, etc.**  
- **Auto sleep / wake-on-motion** using the IMU  
- Powered by a **compact 300 mAh LiPo battery**  
- Built-in **zero-offset and calibration routines**

---

# Component Selection

## Seeed XIAO nRF52840 Sense

- Extremely small and low power
- Built-in **Bluetooth Low Energy**
- Integrated **6-axis IMU** used for cadence detection
- Built-in lithium battery charging support

---

## HX711 Load Cell Amplifier

- Cheap and widely available
- High resolution ADC designed for strain gauge bridges
- Easy microcontroller interface

Commercial power meters typically use more advanced ADCs, but the **HX711 performs well enough for a DIY system**.

---

## BF1K-3AA Strain Gauges

This project uses **BF1K-3AA foil strain gauges**, a commonly used model for precision deformation measurement.

Four gauges are arranged as a **full Wheatstone bridge**, providing:

- Higher sensitivity
- Better temperature stability
- Reduced noise

---

## 300 mAh LiPo Battery

Originally the design planned to use an **18650 cell**, but the final build uses a **300 mAh LiPo battery**.

Advantages:

- Smaller and lighter
- Easier crank-mounted packaging
- Still provides long runtime due to low power consumption

---

## Magnetic Charging Connector

Instead of using USB-C directly, the device charges via a **weatherproof magnetic connector**.

Benefits:

- Resistant to **water, mud, and dirt**
- Protects the small XIAO USB connector
- Easy external charging access

---

# Hardware Setup

Strain gauges are mounted to the **left crank arm** and wired as a **full Wheatstone bridge**.

The bridge connects to the **HX711 load cell amplifier**, which is then connected to the microcontroller.

### HX711 Connections

```
HX711 DOUT → XIAO pin D2
HX711 SCK  → XIAO pin D3
```

The **LSM6DS3 IMU** is built into the XIAO.

Optional:

```
IMU INT1 → XIAO pin D7
```

This can be used for motion-based wake-up.

All electronics operate from the **3.3V rail of the XIAO**.

---

# Software Overview

## Cadence Detection

The onboard **gyroscope** measures crank angular velocity.

Processing steps:

1. Read angular velocity (deg/s)
2. Apply a simple low-pass filter
3. Detect positive → negative → positive zero crossings

Each cycle corresponds to **one crank revolution**.

---

## Power Calculation

Torque is measured through crank deformation.

```
Torque = Force × Crank Length
Power  = Torque × Angular Velocity
```

HX711 raw readings are converted to torque using calibration factors.

---

## BLE Output

The firmware implements the **Bluetooth Cycling Power Profile (CPP)**.

It broadcasts:

- Instantaneous power
- Cumulative crank revolutions
- Event time

This allows cycling computers and apps to recognize the device as a **standard power meter**.

---

## Power Management

Two levels of power saving are implemented.

### Sleep Mode (during riding)

- MCU sleeps between HX711 interrupts (~80 Hz)
- Very low active current consumption

### Deep Sleep (when idle)

After inactivity:

- HX711 powers down
- MCU enters **System OFF**
- IMU continues running motion detection (~10 µA)

Any pedal movement wakes the system.

---

# Calibration

Two calibration steps are required.

---

## 1. Zero Offset (Tare)

With the crank **unloaded**, record the HX711 offset.

This ensures:

```
0 Nm torque at rest
```

---

## 2. Scale Factor (Known Weight)

Hang a known mass from the pedal.

Example:

```
Mass: 10 kg
Crank length: 175 mm

Torque = 10 × 9.81 × 0.175 ≈ 17.15 Nm
```

Record the HX711 counts and compute the conversion factor from **counts → torque**.

Helper functions are included in the firmware to assist with calibration.

---

# Limitations

- **Single-sided measurement (left crank)**  
  The firmware doubles the measured value to estimate total rider power.

- **HX711 precision**  
  Commercial power meters use higher-end ADCs, but the HX711 performs adequately for DIY.

- **Cadence filtering**  
  Currently uses a simple smoothing filter.

---

# License

Open hardware / open source project.  
Feel free to modify, improve, and build your own version.
