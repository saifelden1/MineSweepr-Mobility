# STM32_Mobility ECU: System Requirements & Implementation Specification

**Document Version:** 1.0.0  
**Target Hardware:** STM32F411CEU6 BlackPill (ARM Cortex-M4 @ 96 MHz)  
**RTOS:** FreeRTOS (CMSIS-RTOS v2 API) + micro-ROS Client  
**Project Directory:** `STM32_Mobility/`

---

## 1. Subsystem Scope & Objectives

STM32_Mobility is the real-time execution controller responsible for vehicle locomotion, skid-steer kinematics, IMU orientation telemetry, GPS sentence decoding, and low-latency motor safety watchdog.

---

## 2. Hardware Interfaces & Pin Mapping

### 2.1. 4-Motor Drivetrain (2x Cytron MDD10A Dual-Channel Drivers)
The rover utilizes 4 geared DC motors in a 4WD differential skid-steer configuration.

| Motor Function | Cytron Board & Channel | PWM Peripheral Pin | DIR GPIO Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Front-Left (FL)** | Driver 1, Channel 1 | TIM4_CH1 (`PB6`) | `PB4` | Left side drive |
| **Rear-Left (RL)**  | Driver 1, Channel 2 | TIM4_CH2 (`PB7`) | `PB5` | Left side drive |
| **Front-Right (FR)**| Driver 2, Channel 1 | TIM4_CH3 (`PB8`) | `PC13` | Right side drive |
| **Rear-Right (RR)** | Driver 2, Channel 2 | TIM4_CH4 (`PB9`) | `PC14` | Right side drive |

- **PWM Frequency:** $20\,\text{kHz}$ ultrasonic PWM to eliminate audible motor whine (TIM4 clock @ 96 MHz, $\text{ARR} = 4799, \text{PSC} = 0$).
- **Direction Inversion:** Software macros in BSP allowing trivial per-motor direction polarity flipping.

### 2.2. MPU-6050 6-DOF IMU
- **Bus:** I2C1 (`PB8`/`PB9` remapped or `PB6`/`PB7` alternate, 400 kHz Fast Mode).
- **Sampling Rate:** $50\,\text{Hz}$ synchronous task acquisition.
- **Data Extracted:** Raw accelerometer ($X, Y, Z$) and gyro rates ($\omega_x, \omega_y, \omega_z$), converted to SI units ($\text{m/s}^2$ and $\text{rad/s}$).

### 2.3. NEO-6M GPS Module
- **Bus:** USART1 or USART2 (`PA2`/`PA3` or `PA9`/`PA10` with DMA circular RX buffer).
- **Baud Rate:** 9600 baud (default) or 38400 baud.
- **Parser:** Lightweight, non-blocking NMEA parser extracting latitude, longitude, altitude, and fix status from `$GPGGA` and `$GPRMC` sentences.

### 2.4. SBC Communication (micro-ROS Client)
- **Interface:** USART2 or Native USB CDC.
- **Baud Rate:** $921600\,\text{baud}$ (UART) or 12 Mbps (USB CDC).
- **Transport:** micro-ROS custom serial transport via DMA ring buffers.

---

## 3. Wheel Encoder Status & Kinematics Control

> [!IMPORTANT]
> **NO WHEEL ENCODERS INSTALLED:**  
> The existing codebase contains placeholder hardware encoder drivers (TIM2/TIM3) and closed-loop tick PID algorithms. Because physical encoders are omitted from the build, the firmware must transition to an **open-loop feedforward velocity controller** with optional IMU yaw-rate damping.

### 3.1. Skid-Steer Differential Kinematics
Given target linear velocity $V_x$ and angular velocity $\omega_z$ from `/cmd_vel` ($L = 0.55\,\text{m}$):
$$V_{left} = V_x - \frac{\omega_z \cdot L}{2}$$
$$V_{right} = V_x + \frac{\omega_z \cdot L}{2}$$

### 3.2. Feedforward PWM Mapping
Normalized motor duty cycle $D \in [-1.0, 1.0]$ is derived from linear target speeds:
$$D_{L, R} = \text{clamp}\left(\frac{V_{L, R}}{V_{max}}, -1.0, 1.0\right)$$
$$\text{Duty\_Cycle} = \text{sign}(D) \cdot \left( D_{min\_deadband} + (1.0 - D_{min\_deadband}) \cdot |D| \right)$$

### 3.3. Optional IMU Yaw-Rate Stabilization
When travelling straight ($\omega_{z,\text{cmd}} = 0$), the motor task applies proportional damping based on the MPU-6050 gyro $Z$ to resist sand crabbing:
$$\Delta D = K_p \cdot (0 - \omega_{z,\text{imu}})$$
$$D_{L} \leftarrow D_{L} - \Delta D, \quad D_{R} \leftarrow D_{R} + \Delta D$$

---

## 4. micro-ROS Topic & QoS Configuration

1. **Subscriber:** `/cmd_vel` (`geometry_msgs/msg/Twist`)
   - Callback copies $V_x$ and $\omega_z$ into thread-safe command buffer.
   - Resets the hardware safety watchdog timestamp.
2. **Publisher:** `/imu/data` (`sensor_msgs/msg/Imu` @ 50 Hz)
   - Transmits orientation, angular velocities, and linear accelerations.
3. **Publisher:** `/gps/fix` (`sensor_msgs/msg/NavSatFix` @ 5 Hz)
   - Transmits WGS-84 position and fix validity status.

---

## 5. FreeRTOS Task Architecture

```
Task Name           Priority          Period    Function
--------------------------------------------------------------------------------------
Task_SafetyWDT      osPriorityRealtime 10 ms    Zeroes PWM if cmd_vel age > 200 ms
Task_MotorControl   osPriorityHigh     20 ms    Executes kinematics & writes Cytron PWM
Task_MicroROS       osPriorityNormal   5 ms     Runs micro-ROS agent executor & spin
Task_SensorAcq      osPriorityNormal   20 ms    Reads MPU-6050 I2C & parses GPS NMEA
```

### 5.1. Safety Watchdog Determinism
If no `/cmd_vel` message is received for **$> 200\,\text{ms}$** (e.g. SBC crash, ROS 2 crash, or cable disconnect):
- `Task_SafetyWDT` triggers immediately.
- Writes `CCR = 0` to all 4 TIM4 PWM channels.
- Puts Cytron drivers in passive coast/brake state.

---

## 6. Implementation Checklist for Dedicated Chat

- [ ] **Task 1:** Remove/bypass TIM2 and TIM3 hardware encoder dependencies and encoder PID loops.
- [ ] **Task 2:** Refactor `MDD10A` driver for 4 independent channels (TIM4 CH1-CH4 + 4 DIR pins).
- [ ] **Task 3:** Implement skid-steer feedforward kinematics with deadband compensation.
- [ ] **Task 4:** Finalize MPU-6050 I2C reader and calibrate gyro zero-offsets.
- [ ] **Task 5:** Finalize NEO-6M NMEA parser with DMA ring buffer.
- [ ] **Task 6:** Configure micro-ROS serial transport and publish `/imu/data` and `/gps/fix`.
- [ ] **Task 7:** Validate 200 ms safety watchdog timeout via hardware unit tests.
