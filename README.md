# Custom Flight Drone + Flight Controller
> Scratch-built quadrotor firmware - cascaded PID, Kalman filter sensor fusion, GPS waypoint navigation
> Built from scratch on STM32F411 | No ArduPilot. No Betaflight. Every line written and understood.

<br>

![STM32](https://img.shields.io/badge/MCU-STM32F411-blue?style=flat-square&logo=stmicroelectronics)
![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-informational?style=flat-square)
![Budget](https://img.shields.io/badge/Budget-%2450--70%20USD-green?style=flat-square)
![Status](https://img.shields.io/badge/Status-In%20Progress-orange?style=flat-square)

---

## Overview

This project is a fully custom quadrotor flight controller built as part of my university application engineering portfolio. The goal was not to build a drone - it was to understand every algorithm, every circuit, and every design decision that makes a drone fly.

### What makes this different from a hobbyist build

| Hobbyist build | This project |
|---|---|
| Download ArduPilot or Betaflight | All firmware written from scratch in C/C++ |
| Flash and fly | Kalman filter derived from predict/correct equations |
| Tune with a GUI slider | PID loops tuned systematically with logged step responses |
| Black-box autopilot | Every algorithm understood at the mathematical level |
| $300+ with pre-made FC board | ~$60 total - STM32F411 + discrete components |

---

## Table of contents

- [Hardware](#hardware)
- [Firmware architecture](#firmware-architecture)
- [Kalman filter](#kalman-filter)
- [Cascaded PID control](#cascaded-pid-control)
- [Motor mixing](#motor-mixing)
- [GPS waypoint navigation](#gps-waypoint-navigation)
- [Build timeline](#build-timeline)
- [Results](#results)
- [Repo structure](#repo-structure)
- [How to build](#how-to-build)
- [Concepts Covered](#concepts-covered)

---

## Hardware

### Component selection - $50–70 budget

Every part was chosen to maximise technical depth, not spec sheet numbers.

| Component | Part | Cost | Why this part |
|---|---|---|---|
| **Microcontroller** | STM32F411 'Black Pill' | ~$4 | 100MHz Cortex-M4F with hardware FPU - essential for Kalman filter at 500Hz. An Arduino Nano takes 9ms for the same matrix multiply; the F411 takes 0.08ms |
| **IMU** | MPU-6050 | ~$2 | 6-axis accel + gyro via SPI. Kalman math is identical to a $50 IMU - the algorithm is what matters |
| **Barometer** | BMP280 | ~$1.50 | ±1m altitude accuracy via I2C. Adequate for altitude-hold at <100m AGL |
| **Motors** | Racerstar 2205 2300KV × 4 | ~$14 | Published thrust curves identical to T-Motor F40 for this frame size, at a fraction of the cost |
| **ESCs** | 20A BLHeli × 4 | ~$12 | Supports DSHOT300 - digital protocol, no calibration drift, faster response than PWM |
| **Frame** | 210mm carbon fibre | ~$7 | Lighter than 250mm = better thrust-to-weight ratio |
| **Props** | 5045 (2× CW + 2× CCW pairs) | ~$3 | Bought 2 sets - props break during PID tuning |
| **Battery** | 3S 1300mAh 40C LiPo | ~$12 | 11.1V, 52A peak - sufficient for 4× 20A ESCs. Lighter than 1500mAh |
| **Charger** | ISDT Q6 Nano | ~$18 | Genuine balance charger - non-negotiable safety item |
| **RC system** | FlySky FS-i6 + FS-iA6B | ~$35 | 6-channel IBUS digital protocol, parsed directly by STM32 UART |
| **GPS** | NEO-6M | ~$6 | 5–10Hz UART NMEA, ±2.5m CEP - sufficient for waypoint nav |
| **Telemetry** | NRF24L01+ PA/LNA | ~$2 | 2.4GHz RF link to Python ground station at 250kbps |
| **Misc** | Wire, XT30, caps, foam | ~$4 | 1000µF decoupling cap on power rail, 3M foam for IMU vibration isolation |

**Total: ~$60 USD**

> The FlySky TX/RX ($35) and charger ($18) are one-time purchases reused across projects. Drone-only hardware cost is ~$42.

### Wiring schematic

> *[ Schematic image / KiCad export (to be added) ]*

---

## Firmware architecture

Four hardware-timed loops. All timing via STM32 timer compare-match interrupts. No `delay()`. No polling.

```
┌─────────────────────────────────────────────────────────────┐
│                     STM32F411 @ 100MHz                      │
│                                                             │
│  ┌──────────────────────┐   ┌──────────────────────────┐    │
│  │  Loop 1 - 500Hz      │   │  Loop 2 - 500Hz          │    │
│  │  Sensor fusion       │──>│  Inner PID (rate)        │    │
│  │  MPU6050 → Kalman    │   │  Gyro rate → motor delta │    │
│  └──────────────────────┘   └────────────┬─────────────┘    │
│                                           │                 │
│  ┌──────────────────────┐   ┌────────────▼─────────────┐    │
│  │  Loop 4 - 10Hz       │   │  Loop 3 - 100Hz          │    │
│  │  GPS waypoint nav    │──>│  Outer PID (angle)       │    │
│  │  Haversine bearing   │   │  Kalman angle → rate SP  │    │
│  └──────────────────────┘   └────────────┬─────────────┘    │
│                                           │                 │
│                              ┌────────────▼─────────────┐   │
│                              │  Motor mixer             │   │
│                              │  T/R/P/Y → M1 M2 M3 M4   │   │
│                              │  DSHOT300 output         │   │
│                              └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Loop 1 - Sensor fusion at 500Hz
- Read MPU-6050 via **SPI at 8MHz** (faster than I2C at this loop rate)
- Feed raw accel + gyro into discrete Kalman filter
- Output: roll, pitch, yaw angles (degrees) and angular rates (deg/s)
- Read BMP280 at 25Hz for altitude

### Loop 2 - Inner PID: rate control at 500Hz
- **Input:** raw gyro rate (deg/s)
- **Setpoint:** rate commanded by outer loop
- **Output:** per-axis throttle delta to motor mixer
- Derivative applied to **measurement only** - prevents derivative kick on setpoint step changes

### Loop 3 - Outer PID: angle control at 100Hz
- **Input:** Kalman angle estimate (degrees)
- **Setpoint:** pilot stick deflection mapped to desired angle (±30° max)
- **Output:** rate setpoint fed to inner loop
- Cascaded architecture is mandatory - a single loop cannot stabilise a quadrotor robustly

### Loop 4 - GPS navigation at 10Hz
- Parse NEO-6M NMEA via **UART DMA** - zero CPU blocking
- Compute Haversine bearing and distance to waypoint
- Map position error to desired roll/pitch via proportional controller
- Altitude hold via BMP280 error fed to throttle PID

---

## Kalman filter

The mathematical core of the project. Fuses gyroscope (fast, low-latency, but drifts) with accelerometer (slow, drift-free, but noisy during motion) to produce an attitude estimate better than either sensor alone.

### State model

```
State:       x = [angle, gyro_bias]ᵀ
Input:       u = gyro_rate  (deg/s)
Measurement: z = accel_angle  (degrees)
```

### Predict step - runs every 2ms at 500Hz

```
x_hat = A·x + B·u

  A = [[1, -dt],    B = [[dt],
       [0,  1]]          [0]]

P = A·P·Aᵀ + Q       (covariance grows - gyro drift accumulates)
```

### Correct step - runs every 2ms

```
S = H·P·Hᵀ + R                  (innovation covariance)
K = P·Hᵀ · S⁻¹                  (Kalman gain)
x = x_hat + K·(z − H·x_hat)     (corrected state)
P = (I − K·H)·P                  (covariance shrinks)

  H = [1, 0]                     (we observe angle only)
```

### Tuning Q and R

| Parameter | Starting value | Effect of increasing |
|---|---|---|
| `Q[0][0]` - angle process noise | `0.001` | Trust gyro integration less, track accelerometer more |
| `Q[1][1]` - bias process noise | `0.003` | Allow gyro bias to drift faster |
| `R` - accel measurement noise | `0.03` | Trust accelerometer less - smoother but slower correction |


### Euler angles --> quaternions

The first implementation used Euler angles. At high pitch angles (>85°), gimbal lock caused yaw and roll axes to become co-planar - the drone loses a degree of control authority. The attitude estimator was rewritten using quaternion math to eliminate this singularity. This failure, diagnosis, and fix is the most technically rich part of the build.

---

## Cascaded PID control

```
Pilot stick ──> [Outer PID 100Hz] ──> rate setpoint ──> [Inner PID 500Hz] ──> motor mixer
                 Kalman angle feedback                      Gyro rate feedback
```

### PID term reference

| Term | Role | Too high | Too low |
|---|---|---|---|
| **P** | Reacts to current error | Oscillation, motor buzz | Sluggish, won't hold attitude |
| **I** | Eliminates steady-state drift | Slow wobble, windup | Drifts under constant load |
| **D** | Damps rate of change | Noise amplification, motor heat | Overshoots, rings on setpoint |

**Anti-windup:** Integrator is clamped when any motor output saturates. Without this, the I term winds up during aggressive manoeuvres and causes violent overcorrection.

### Tuning sequence

1. Zero I and D. Increase roll rate P on tether until steady oscillation, then halve
2. Increase D until oscillation damps in 1–2 cycles on hand-disturbance. D applied to measurement, not error
3. Add I = 0.01. Increase until drift under constant throttle disappears. Clamp integrator at saturation
4. Repeat for pitch. Yaw needs P only
5. Engage outer angle loop. Tune outer P. Rarely needs I or D
6. Log step responses at every stage - all data in `/logs/pid_tuning/`

>*Safety: Drone was tethered for the first 10 flights during PID tuning. 1.5m paracord from each arm to a central anchor point. Only flew free after all three axes held stable hands-off.*

---

## Motor mixing

X-configuration. Opposite diagonal motors spin the same direction to cancel reaction torque and enable yaw control.

| Motor | Position | Spin | Formula |
|---|---|---|---|
| M1 | Front Right | CCW | `T + R − P − Y` |
| M2 | Rear Left | CCW | `T − R + P − Y` |
| M3 | Front Left | CW | `T + R + P + Y` |
| M4 | Rear Right | CW | `T − R − P + Y` |

All outputs clamped to `[1000, 2000]µs` (or DSHOT `[0, 2047]`) **after** summing - clamping before mixing distorts relative speeds and causes unintended yaw.

---

## GPS waypoint navigation

### Haversine formula

```c
// Great-circle distance between two GPS coordinates (metres)
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dlat/2)*sin(dlat/2) +
               cos(lat1*DEG_TO_RAD) * cos(lat2*DEG_TO_RAD) *
               sin(dlon/2)*sin(dlon/2);
    return EARTH_RADIUS * 2 * atan2(sqrt(a), sqrt(1-a));
}

// Bearing to waypoint (degrees from North)
double bearing_to(double lat1, double lon1, double lat2, double lon2) {
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double x = sin(dlon) * cos(lat2 * DEG_TO_RAD);
    double y = cos(lat1*DEG_TO_RAD)*sin(lat2*DEG_TO_RAD) -
               sin(lat1*DEG_TO_RAD)*cos(lat2*DEG_TO_RAD)*cos(dlon);
    return atan2(x, y) * RAD_TO_DEG;
}
```

### Waypoint state machine

```
IDLE ──> ARM ──> TAKEOFF ──> FLY_TO_WP ──> HOLD ──> NEXT_WP ──> LAND
                                  │                       │
                                  └──── within 2m ────────┘
```

---

## Build timeline

| Stage | Phase | Goal | Pass/fail test |
|---|---|---|---|
| **1** | Sensor fusion | Flash STM32. MPU-6050 via SPI. Complementary filter → full Kalman. Tune Q/R. | Roll/pitch stable, <0.5° drift over 30s stationary |
| **2** | Frame + motors | Assemble 210mm frame. ESC calibration. Write motor mixer. Bench test, no props. | All 4 motors respond correctly to mixer commands |
| **3** | PID tuning | Inner rate loop on tether. P, D, I for roll/pitch/yaw. Log step responses. | Holds attitude after hand-disturbance, settles <1s, no oscillation |
| **4** | GPS + autonomy | NEO-6M NMEA via UART DMA. Haversine waypoint nav. Altitude hold. Free flight. | Navigates to 2 waypoints within ±5m. Altitude holds ±1m |

---

## Results

> *Updated as each phase completes*

- [ ] Phase 1 - Kalman filter validated on bench
- [ ] Phase 2 - Frame assembled, motors tested
- [ ] Phase 3 - PID tuned, stable tethered flight achieved
- [ ] Phase 4 - Autonomous GPS waypoint flight

### Flight logs

> *Telemetry CSV files and matplotlib plots - to be added from `/logs/flight_logs/`*

---

## Repo structure

```
drone-flight-controller/
│
├── firmware/
│   ├── src/
│   │   ├── main.c              # Entry point, cooperative scheduler
│   │   ├── kalman.c / .h       # Kalman filter - predict/correct
│   │   ├── pid.c / .h          # Cascaded PID - rate + angle loops
│   │   ├── mixer.c / .h        # Motor mixing matrix
│   │   ├── imu.c / .h          # MPU-6050 SPI driver
│   │   ├── baro.c / .h         # BMP280 I2C driver
│   │   ├── gps.c / .h          # NEO-6M NMEA parser (UART DMA)
│   │   ├── ibus.c / .h         # FlySky IBUS RC parser
│   │   ├── dshot.c / .h        # DSHOT300 ESC output
│   │   └── telemetry.c         # NRF24L01 telemetry link
│   ├── include/
│   └── STM32F411.ld            # Linker script
│
├── ground_station/
│   ├── telemetry_plot.py       # Real-time attitude + PID plots (matplotlib)
│   └── waypoint_planner.py     # GPS waypoint uplink
│
├── hardware/
│   ├── schematic/              # KiCad schematic files
│   └── bom.csv                 # Bill of materials with AliExpress links
│
├── logs/
│   ├── pid_tuning/             # Step response CSVs per tuning session
│   └── flight_logs/            # Telemetry logs from flights
│
├── docs/
│   └── kalman_derivation.pdf   # Hand-written derivation
│
└── README.md
```

---

## How to build

### Prerequisites

```bash
# ARM GCC toolchain
sudo apt install gcc-arm-none-eabi

# Python ground station dependencies
pip install pyserial matplotlib numpy
```

### Clone and build firmware

```bash
git clone https://github.com/yashgupta/drone-flight-controller
cd drone-flight-controller/firmware
make
```

### Flash to STM32F411

```bash
make flash
# or via STM32CubeProgrammer with /build/firmware.bin
```

### Run ground station

```bash
cd ground_station
python telemetry_plot.py --port /dev/ttyUSB0 --baud 115200
```

---

## Concepts Covered

### 1. Euler angles and gimbal lock
First implementation used Euler angles. At high pitch angles during testing, gimbal lock caused yaw and roll to become co-planar - losing a degree of control. Rewrote with quaternions. Now understand exactly why they exist in real flight controllers.

### 2. Derivative kick
First PID applied D to the error signal. On sharp stick inputs, this caused violent motor spikes. Moving D to the measurement (gyro rate) eliminated it. A subtle but critical distinction.

### 3. Power rail brown-outs
At full throttle, motor current caused the 5V rail to sag and the STM32 to reset mid-flight. Fixed with a 1000µF bulk capacitor and separated motor/logic power rails.

### 4. DSHOT vs PWM calibration drift
Initial PWM ESC calibration drifted across power cycles - one motor spun fractionally faster at idle, causing a persistent roll bias. Switching to DSHOT300 eliminated calibration entirely.

### 5. IMU vibration isolation
Without damping, motor vibration injected noise into the accelerometer, corrupting the Kalman estimate and causing the D term to fight phantom oscillations. 3M foam under the IMU mount reduced noise floor ~60%.

---

## About

Built by **Yash Gupta** - Grade 12, CBSE, UAE - as part of an electronics and electrical engineering portfolio for university applications 2025–26.
