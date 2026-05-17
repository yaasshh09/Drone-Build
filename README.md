# Custom Drone
**Scratch-built quadrotor firmware | PID control, Kalman filter, GPS waypoint nav | optimised for $50–70 USD**

---

## Project metadata

| Field | Detail |
|---|---|
| **Project type** | Embedded Firmware + Control Systems + Autonomous Robotics |
| **Field** | Aerospace Engineering / Embedded Systems / Control Theory |
| **Difficulty** | Advanced - all firmware written from scratch, no ArduPilot or Betaflight |
| **Duration** | 4 months (can run parallel to Bionic Hand from Month 3 onward) |
| **Hard budget** | $50–70 USD MAXIMUM - every component selected for maximum technical depth per dollar |
| **MCU** | STM32F411 'Black Pill' - 100MHz Cortex-M4F with hardware FPU |
| **Key constraint** | Custom firmware only. Using any pre-written flight controller stack disqualifies the project entirely |

---

## Bill of materials - maximum capability for $50–70

| Component | What to buy & why | Source | Cost |
|---|---|---|---|
| **STM32F411 Black Pill** | 100MHz Cortex-M4F, hardware FPU - essential for Kalman at 500Hz. 8x timers for PWM. Best sub-$5 flight controller MCU available. | AliExpress | ~$4 |
| **MPU-6050 IMU** | 6-axis accel + gyro, SPI/I2C. Sufficient noise floor for Kalman fusion at this scale. Well-documented datasheet. Kalman math is identical to a $50 IMU. | AliExpress | ~$2 |
| **BMP280 barometer** | ±1m altitude accuracy. I2C. Adequate for altitude-hold and waypoint nav at <100m. | AliExpress | ~$1.50 |
| **Racerstar 2205 2300KV x4** | Well-characterised thrust curves available online. 5" prop compatible. Sufficient for 210mm frame. | AliExpress | ~$14 |
| **20A BLHeli ESC x4** | BLHeli firmware, supports DSHOT300. 20A sufficient for 5" props on 3S. Calibrate all 4 to identical endpoints. | AliExpress | ~$12 |
| **210mm carbon frame** | Lighter than 250mm = lower motor load = longer flight time. Pre-drilled M3 motor mounts. | AliExpress | ~$7 |
| **5045 props x4 pairs** | 2 CW + 2 CCW pairs. Buy 2 sets - you will break props during PID tuning. | AliExpress | ~$3 |
| **3S 1300mAh 40C LiPo** | 11.1V nominal, 40C discharge = 52A peak - sufficient for 4x 20A ESCs. Lighter than 1500mAh = better thrust/weight. | Local/online | ~$12 |
| **LiPo balance charger** | ISDT Q6 Nano or SkyRC e430 - genuine balance charger mandatory. Do not substitute with a cheap unbalanced charger. Fire risk. | Local/online | ~$18 |
| **FlySky FS-i6 TX + FS-iA6B RX** | 6-channel RC, IBUS digital protocol - cleaner than PWM, parsed directly by STM32 UART. | AliExpress | ~$35 |
| **NEO-6M GPS module** | Budget GPS, 5–10Hz, UART NMEA output. Accuracy ±2.5m CEP - sufficient for waypoint nav proof-of-concept. | AliExpress | ~$6 |
| **NRF24L01+ PA/LNA** | 2.4GHz RF telemetry to laptop. 250kbps - more than enough for attitude/PID telemetry logging. | AliExpress | ~$2 |
| **Misc (wire, XT30, caps)** | XT30 connectors, 16AWG silicone wire, 1000uF decoupling cap on power rail, 3M foam for IMU vibration damping. | AliExpress | ~$4 |

**Running total:** ~$120 list above, BUT the TX/RX and LiPo charger are one-time purchases reused across projects. Net drone-only spend is ~$50 - 65 USD.

If you already own a charger or RC system, you come in well under $50.

---


## Firmware architecture - STM32F411

The flight controller runs four hardware-timed loops at fixed rates. All timing uses STM32 hardware timer compare - match interrupts, never `delay()` or polling. The F411's hardware FPU executes the Kalman filter matrix operations in ~0.08ms, making 500Hz sensor fusion feasible.

### Loop 1 : Sensor fusion at 500Hz (TIM2 interrupt)
Read MPU-6050 via SPI at 8MHz. Feed raw accel + gyro into discrete Kalman filter. Output: roll, pitch, yaw angles in degrees and rates in deg/s. Read BMP280 via I2C at 25Hz for altitude estimate.

### Loop 2 : Inner PID: rate control at 500Hz
- **Input:** gyro rate (deg/s)
- **Setpoint:** from outer loop
- **Output:** throttle delta per axis
- Derivative applied to gyro measurement only - NOT to error - to prevent derivative kick when setpoint changes abruptly

### Loop 3 - Outer PID: angle control at 100Hz
- **Input:** Kalman angle estimate (degrees)
- **Setpoint:** pilot stick deflection mapped to desired angle (max ±30°)
- **Output:** rate setpoint fed to inner loop
- Cascaded architecture is not optional - a single loop cannot stabilise attitude robustly

### Loop 4 - GPS navigation at 10Hz
Parse NEO-6M NMEA sentences via UART DMA. Compute Haversine bearing and distance to waypoint. Map to desired roll/pitch angle via proportional position controller. Altitude hold via BMP280 feedback to throttle PID.

### Motor mixer - runs after every inner loop cycle
Translates throttle/roll/pitch/yaw commands to 4 motor speeds via X-config mixing matrix. Outputs clamped to [1000, 2000]µs equivalent. DSHOT300 preferred over PWM - digital protocol, no ESC calibration drift, faster response.

---

## Kalman filter - full derivation (must understand)

The Kalman filter fuses the gyroscope (fast, low-latency, but drifts) with the accelerometer (slow, drift-free, but noisy) to produce an attitude estimate better than either sensor alone. This is the mathematical centrepiece of the project. You must derive it, implement it, tune it, and explain it from first principles in an interview.

### State vector and system model

```
State:   x = [angle, gyro_bias]^T     - angle in degrees, bias drift in deg/s
Input:   u = gyro_rate                - raw gyro reading in deg/s
Measurement: z = accel_angle          - angle computed from accelerometer
```

### Predict step (runs every 2ms at 500Hz)

```
x_hat = A*x + B*u       where A = [[1, -dt], [0, 1]],  B = [[dt], [0]]
P     = A*P*A^T + Q     covariance grows - gyro drift accumulates
```

### Correct step (runs every 2ms)

```
S = H*P*H^T + R          innovation covariance
K = P*H^T * S^-1         Kalman gain - how much to trust accel vs gyro
x = x_hat + K*(z - H*x_hat)   corrected state estimate
P = (I - K*H)*P          covariance shrinks after update
```

### Tuning Q and R matrices

| Parameter | Start value | Effect of increasing |
|---|---|---|
| Q[0][0] - angle process noise | 0.001 | Trust gyro integration less - estimate tracks accel more |
| Q[1][1] - bias process noise | 0.003 | Allow bias to change faster |
| R - accel measurement noise | 0.03 | Trust accelerometer less - smoother but slower correction |


---

## PID control - cascaded loops and tuning sequence

Two cascaded PID loops control attitude. The inner loop runs at 500Hz on gyro rate. The outer loop runs at 100Hz on Kalman angle. Tune inner loop first - always. Never attempt to tune both simultaneously.

### PID term reference

| Term | What it controls | Too high | Too low |
|---|---|---|---|
| **P** | Reacts to current error | Oscillation, buzzing motors | Sluggish, won't hold attitude |
| **I** | Eliminates steady-state drift | Slow wobble, integrator windup | Drifts under constant load |
| **D** | Damps the rate of change | Noise amplification, motor heat | Overshoots, rings on setpoint |

### Tuning sequence - follow this exactly

1. Zero all I and D. Increase P (roll rate) on tether until drone oscillates steadily, then halve it
2. Increase D slowly until oscillation damps within 1–2 cycles on hand-disturbance test. Apply D to gyro measurement only - not error - to avoid derivative kick
3. Add small I (start 0.01) until slow drift under constant throttle disappears. Implement anti-windup: clamp integrator when any motor output saturates
4. Repeat steps 1–3 for pitch. Yaw rate usually needs only P
5. Engage outer angle loop. Tune outer P until angle holds correctly. Rarely needs I or D
6. Log step responses at every stage via NRF24 telemetry. This data is your technical evidence

>Tether the drone for the first 8–10 flights during PID tuning. Tie 1.5m of paracord to each arm, anchored to a central point. Only remove tether after all three axes hold stable.

---

## Motor mixing - quadrotor X-configuration

Translates throttle (T), roll (R), pitch (P), yaw (Y) commands into 4 motor speeds. All outputs clamped to [1000, 2000]µs after summing - clamp after mixing, not before.

| Motor | Spin direction | Speed formula |
|---|---|---|
| M1 - Front Right | CCW | `T + R − P − Y` |
| M2 - Rear Left | CCW | `T − R + P − Y` |
| M3 - Front Left | CW | `T + R + P + Y` |
| M4 - Rear Right | CW | `T − R − P + Y` |


---


### Engineering process skills
- Autonomous systems - GPS Haversine waypoint navigation
- Safety engineering - arming sequence, kill-switch, tethered testing protocol
- Data logging and analysis - step response plots, live telemetry
- Budget engineering - maximum technical depth per dollar spent
- Systematic debugging - isolating control vs mechanical vs electrical faults
