# Self-Balancing Robot (STM32F411RE)

Two-wheeled self-balancing robot (inverted pendulum) built on an **STM32F411RE**. It estimates the tilt angle with an **MPU9250** IMU and a **Kalman filter**, and keeps its balance with a **cascaded PID control** (angle loop + wheel-speed loop + position correction), all running on bare-metal C with the STM32 HAL.

<img width="1288" height="1287" alt="image" src="https://github.com/user-attachments/assets/6874a2c1-a9b3-4c84-b3ce-ce3a7ea92fa8" />
<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/852b48e6-26e3-490a-8333-eb24904af12f" />
<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/0d903a93-36f2-4a5f-a9d1-529a4a9a3637" />
<img width="716" height="1600" alt="image" src="https://github.com/user-attachments/assets/09d50ba6-06e1-41de-91fc-da986cb81f47" />


## Features

- Tilt estimation with a 2-state **Kalman filter** (angle + gyroscope bias), fusing accelerometer and gyroscope.
- **Non-blocking IMU reading**: I2C at 400 kHz with DMA, so the CPU is never stalled waiting for the sensor.
- **Cascaded control** executed in a fixed 10 ms timer interrupt (TIM5):
  1. Position loop: keeps the robot from drifting by shifting the angle setpoint.
  2. Angle loop (PD): converts tilt error into a wheel-speed setpoint.
  3. Speed loop (PI, one per motor): tracks the wheel-speed setpoint using encoder feedback.
- Quadrature **encoder reading** with hardware timers (TIM1 and TIM2 in encoder mode).
- 1 kHz **PWM** (1000 steps) on TIM3 to drive both motors in both directions.
- Serial telemetry at 115200 baud (`printf` over USART2): sample count, Kalman angle and control outputs.

## Hardware

| Component | Details |
|---|---|
| Microcontroller | STM32F411RE, 84 MHz (HSI + PLL) |
| IMU | MPU9250 (accelerometer ±4 g, gyroscope ±500 °/s), I2C address 0x68 |
| Motor driver | L298N dual H-bridge *(confirm the model)* |
| Motors | 2 DC gear motors with quadrature encoders *(add model and rated RPM)* |
| Power | LiPo battery, 2200 mAh *(add cell count)* + 5 V converter for the logic |
| Wiring | Jumper wires |

### Pinout

| Function | Pin | Peripheral |
|---|---|---|
| Motor 1 PWM (IN1 / IN2) | PA6 / PA7 | TIM3 CH1 / CH2 |
| Motor 2 PWM (IN3 / IN4) | PB0 / PB1 | TIM3 CH3 / CH4 |
| Encoder 1 (A / B) | PA8 / PA9 | TIM1 (encoder mode) |
| Encoder 2 (A / B) | PA0 / PA1 | TIM2 (encoder mode) |
| MPU9250 SCL / SDA | PB6 / PB7 | I2C1 (400 kHz) |
| Serial telemetry TX / RX | PA2 / PA3 | USART2 (115200 baud) |

## How it works

### 1. Angle estimation (Kalman filter)

The pitch angle computed from the accelerometer is noisy, and the gyroscope drifts over time. The Kalman filter (`Core/Src/kalman.c`) combines both:

- **State:** angle and gyroscope bias.
- **Prediction:** the angle is propagated with the gyroscope rate minus the estimated bias.
- **Correction:** the accelerometer angle corrects the prediction and updates the bias estimate.

Tuning parameters (`Core/Inc/kalman.h`):

| Parameter | Value |
|---|---|
| `KALMAN_Q_ANGLE` | 0.001 |
| `KALMAN_Q_BIAS` | 0.003 |
| `KALMAN_R_MEASURE` | 0.03 |

The sensor is read with I2C DMA: when a transfer completes, the main loop parses the data, updates the filter with the real elapsed `dt`, and requests the next reading. The control interrupt only reads the latest angle.

### 2. Control (cascaded loops, every 10 ms)

```
position ──► angle setpoint ──► [PD angle] ──► wheel-speed setpoint ──► [PI speed x2] ──► PWM
(encoders)     (limited ±3°)    (from Kalman)       (limited ±8 rad/s)     (encoders)
```

| Loop | Type | Gains |
|---|---|---|
| Position → angle setpoint | P | `Kp_pos = 0.1` (setpoint limited to ±3°) |
| Angle | PD | `Kp_ang = 6.0`, `Kd_ang = 0.1`, 0.5° dead zone |
| Wheel speed (each motor) | PI | `Kp = 50`, `Ki = 300`, with anti-windup |

The speed loop output is saturated to ±1000 (PWM range). The encoders give 2π/2400 rad per count, which is used to compute wheel speed and position.

## Project structure

```
Core/
  Inc/         Headers (kalman.h, mpu9250.h, main.h, ...)
  Src/
    main.c       Initialization, Kalman loop and control ISR (TIM5)
    kalman.c     Kalman filter
    mpu9250.c    MPU9250 driver (I2C + DMA)
Drivers/       STM32 HAL / CMSIS
BalancinFinal.ioc   STM32CubeMX configuration
```

## Build and flash

1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone the repository:
   ```bash
   git clone https://github.com/fatimajmzj/self-balancing-robot-stm32f411re.git
   ```
3. In STM32CubeIDE: **File → Import → Existing Projects into Workspace** and select the cloned folder.
4. Build the project and flash it with an ST-LINK (**Run → Debug / Run**).
5. Open a serial terminal at 115200 baud to see the telemetry.

> Keep the robot upright and still while it powers on: the initial angle of the filter is taken from the first accelerometer reading.

## Results

*Add here a video/GIF of the robot balancing and, if you have it, a plot comparing the raw accelerometer angle against the Kalman estimate.*

## Possible improvements

- Tune the gains with a systematic method and document the results.
- Add a remote control (Bluetooth/RF) to drive the robot.
- Use the magnetometer of the MPU9250 for heading control.

## License

See the [LICENSE](LICENSE) file.
