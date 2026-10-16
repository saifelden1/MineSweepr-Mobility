# STM32_Mobility Firmware

> **Current flash wiring — 16 October 2026:** use the table below and `STM32_Mobility.ioc`. The older indicator diagrams and PC13/PC14 motor assignments later in this historical guide are superseded. Build with `cmake -S . -B build-relay -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc.cmake` then `cmake --build build-relay`. Flash `build-relay/STM32_Mobility.elf` (or `.bin` at `0x08000000`).

| Black Pill pin | Connected device pin |
|---|---|
| PB6 / PB4 | Left MDD10A PWM1 / DIR1 (front-left) |
| PB7 / PB5 | Left MDD10A PWM2 / DIR2 (rear-left) |
| PB8 / PA4 | Right MDD10A PWM1 / DIR1 (front-right) |
| PB9 / PA5 | Right MDD10A PWM2 / DIR2 (rear-right) |
| PB10 / PB3 | MPU-6050 SCL / SDA (3.3 V pull-ups) |
| 3V3 / GND | Compatible MPU-6050 supply / ground; AD0 to GND |
| PA3 / PA2 | GPS TX / RX (9600 baud; 3.3 V logic; RX optional) |
| PA9 TX | Pi physical pin 10 / GPIO15 RX |
| PA10 RX | Pi physical pin 8 / GPIO14 TX |
| GND | Pi physical pin 6; both motor-driver grounds; GPS/IMU grounds |
| 5V / GND | Regulated 5 V board supply / return |
| PA13 / PA14 | ST-Link SWDIO / SWCLK (programming) |

USART1 remains 921600 baud, ROS domain 42. The Pi agent uses its already configured GPIO UART `/dev/ttyAMA0`. No Black Pill relay, status LEDs or buzzer are driven. The 300 ms motor watchdog and both rear motor inversions are retained. Power the GPS breakout from its rated supply, and power motors through the drivers, not the MCU. Aux uses a separate USB–UART adapter connected to Pi USB; do not join the two STM TX outputs.

## Autonomous Minesweeper Rover — Locomotion, Navigation & micro-ROS Control Subsystem

![Platform](https://img.shields.io/badge/Platform-STM32F411CEU6%20BlackPill-blue.svg)
![Clock](https://img.shields.io/badge/SYSCLK-96%20MHz%20(25MHz%20HSE)-brightgreen.svg)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS%20CMSIS__V2-orange.svg)
![Middleware](https://img.shields.io/badge/Middleware-micro--ROS%20Humble-purple.svg)
![Baudrate](https://img.shields.io/badge/UART%20DMA-921600%20Baud-yellow.svg)
![Tests](https://img.shields.io/badge/Tests-61%2F61%20Passing-success.svg)

> **Dedicated Hardware & STM32CubeMX Setup Guide:**  
> For complete register-level and graphical configuration instructions in STM32CubeMX (clock tree math, pin alternate functions, timer ARR/PSC settings, and IDE properties), consult [**`CUBEMX_CONFIGURATION_GUIDE.md`**](CUBEMX_CONFIGURATION_GUIDE.md).

---

## Table of Contents

1. [Executive Summary & System Architecture](#1-executive-summary--system-architecture)
2. [Hardware Pinout & Complete Wiring Guides](#2-hardware-pinout--complete-wiring-guides)
   - [2.1. 40-Pin BlackPill Pinout Diagram](#21-40-pin-blackpill-pinout-diagram)
   - [2.2. Drivetrain & Motor Driver Interconnect (Cytron MDD10A)](#22-drivetrain--motor-driver-interconnect-cytron-mdd10a)
   - [2.3. Inertial Measurement Unit (MPU-6050 6-DOF IMU)](#23-inertial-measurement-unit-mpu-6050-6-dof-imu)
   - [2.4. Global Positioning System (u-blox NEO-6M GPS)](#24-global-positioning-system-u-blox-neo-6m-gps)
   - [2.5. Diagnostic Indicators Subsystem (Buzzer & 3 Status LEDs)](#25-diagnostic-indicators-subsystem-buzzer--3-status-leds)
   - [2.6. micro-ROS Host Link & ST-Link SWD Programming](#26-micro-ros-host-link--st-link-swd-programming)
   - [2.7. Critical Pin Multiplexer Conflict Resolution (TIM4 vs I2C1)](#27-critical-pin-multiplexer-conflict-resolution-tim4-vs-i2c1)
3. [micro-ROS Communication Contract](#3-micro-ros-communication-contract)
   - [3.1. Topic Routing & Interface Specifications](#31-topic-routing--interface-specifications)
   - [3.2. Configurable Baud Rate Serial Transport Layer](#32-configurable-baud-rate-serial-transport-layer)
4. [Task Execution Flow & Safety Architecture](#4-task-execution-flow--safety-architecture)
   - [4.1. Rate Monotonic Scheduling (RMS) Hierarchy](#41-rate-monotonic-scheduling-rms-hierarchy)
   - [4.2. 200 ms Safety Watchdog Cutoff & Mine Trigger Interlock](#42-200-ms-safety-watchdog-cutoff--mine-trigger-interlock)
   - [4.3. Skid-Steer Kinematics, Polarity & Deadband Compensation](#43-skid-steer-kinematics-polarity--deadband-compensation)
   - [4.4. Closed-Loop Gyro Yaw Damping in Loose Terrain](#44-closed-loop-gyro-yaw-damping-in-loose-terrain)
5. [Build, Flash & Host Verification](#5-build-flash--host-verification)
   - [5.1. STM32CubeIDE Build Instructions](#51-stm32cubeide-build-instructions)
   - [5.2. ST-Link SWD Flashing](#52-st-link-swd-flashing)
   - [5.3. micro-ROS Agent Host Execution](#53-micro-ros-agent-host-execution)
   - [5.4. Automated Test Suite Execution (pytest)](#54-automated-test-suite-execution-pytest)
6. [Repository Structure & Code Layout](#6-repository-structure--code-layout)

---

## 1. Executive Summary & System Architecture

`STM32_Mobility` is the real-time embedded mobility and navigation controller of the Autonomous Minesweeper Rover. Running on an ARM Cortex-M4F STM32F411CEU6 clocked at **96.0 MHz**, it executes deterministic FreeRTOS CMSIS_V2 tasks to drive a 4-motor skid-steer chassis, sample navigation telemetry, enforce failsafe motor cutoffs, and bridge directly into ROS 2 Humble via micro-ROS over UART DMA.

```
                    Host SBC: Raspberry Pi 5 (ROS 2 Humble)
                                       │
              USART1 DMA @ 921600 Baud │ (/cmd_vel, /imu/data, /gps/fix, /stm32_heartbeat)
                                       ▼
       ┌──────────────────────────────────────────────────────────────┐
       │             STM32_Mobility ECU (STM32F411CEU6)               │
       │        96 MHz SYSCLK, 64 KB FreeRTOS Heap (heap_4)           │
       ├──────────────────────────────────────────────────────────────┤
       │ FreeRTOS CMSIS_V2 Real-Time Task Hierarchy:                  │
       │  1. Task_SafetyWDT  (Realtime, 100 Hz): 200 ms cutoff        │
       │  2. Task_MotorCtrl  (High, 50 Hz): Skid-steer kinematics     │
       │  3. Task_SensorAcq  (AboveNormal, 50 Hz): IMU & GPS parsing  │
       │  4. Task_MicroROS   (Normal, Spin): micro-ROS client         │
       └───────┬──────────────┬──────────────┬──────────────┬─────────┘
               │              │              │              │
               ▼              ▼              ▼              ▼
       Dual Cytron MDD10A  MPU-6050       u-blox NEO-6M  Diagnostics
       4-Motor Drive       6-DOF IMU      GPS Receiver   Subsystem
       PWM: TIM4 CH1-CH4   I2C2 Fast Mode USART2 DMA     PB12: Buzzer
       (PB6, PB7, PB8, PB9)(PB10 SCL,     (PA2 TX,       PB13: Red LED (WDT)
       DIR: PB4, PB5,      PB3 SDA        PA3 RX         PB14: Yellow LED (Motion)
            PC13, PC14     @ 400 kHz)     @ 9600 Baud)   PB15: Green LED (ROS OK)
```

### Key Engineering Features:
- **Ultrasonic Locomotion (20 kHz PWM)**: Silent, high-efficiency motor control over `TIM4` Channels 1–4 with 4800 discrete steps of resolution.
- **Fail-Safe Watchdog Interlock**: Independent real-time task halts all motors within $< 200\,\text{ms}$ if ROS 2 communication drops or if a mine trigger is asserted.
- **Conflict-Free Hardware Pinout**: MPU-6050 routed to `I2C2` (`PB10`/`PB3`), eliminating the classic STM32F411 pin collision between `TIM4` and `I2C1`.
- **Zero-Copy DMA Circular Buffers**: `USART1` (micro-ROS) and `USART2` (GPS) utilize circular DMA reception to prevent packet loss under heavy telemetry bursts.
- **Modular Hardware Abstraction Layer**: Centralized pin macros in `Core/Inc/mobility_pin_config.h` allow remapping pins without touching application or driver code.

---

## 2. Hardware Pinout & Complete Wiring Guides

### 2.1. 40-Pin BlackPill Pinout Diagram

The WeAct Studio STM32F411CEU6 BlackPill physical pin layout (top view):

```
                              ┌────────────────┐
                              │  USB-C CONNECTOR
                              └────────────────┘
                     [B12]  1 ──              ── 40  [VBAT]
                     [B13]  2 ──              ── 39  [C13] (FR DIR)
                     [B14]  3 ──              ── 38  [C14] (RR DIR)
                     [B15]  4 ──              ── 37  [C15]
                      [A8]  5 ──  STM32F411   ── 36  [NRST] (Reset)
                      [A9]  6 ──  BlackPill   ── 35  [A0]
                     [A10]  7 ──  (Top View)  ── 34  [A1]
                     [A11]  8 ──              ── 33  [A2]  (GPS TX)
                     [A12]  9 ──  ARM 96 MHz  ── 32  [A3]  (GPS RX)
                     [A15] 10 ──              ── 31  [A4]
                      [B3] 11 ──              ── 30  [A5]
                      [B4] 12 ──              ── 29  [A6]
                      [B5] 13 ──              ── 28  [A7]
                      [B6] 14 ──              ── 27  [B0]
                      [B7] 15 ──              ── 26  [B1]
                      [B8] 16 ──              ── 25  [B2]  (BOOT1)
                      [B9] 17 ──              ── 24  [B10] (IMU SCL)
                      [5V] 18 ──              ── 23  [3V3]
                     [GND] 19 ──              ── 22  [GND]
                     [3V3] 20 ──              ── 21  [5V]
                              └────────────────┘
                          [ 3V3  SWDIO  SWCLK  GND ]
                               SWD DEBUG HEADER
```

---

### 2.2. Drivetrain & Motor Driver Interconnect (Cytron MDD10A)

The drivetrain utilizes two **Cytron MDD10A Dual-Channel 10A DC Motor Drivers** to power 4 high-torque planetary gearmotors:
- **Driver 1 (Left Track)**: Front-Left (FL) and Rear-Left (RL) motors.
- **Driver 2 (Right Track)**: Front-Right (FR) and Rear-Right (RR) motors.

| BlackPill Pin | Net Name | Function / Alternate Function | Target Device Pin | Functional Role |
| :---: | :---: | :---: | :---: | :--- |
| **PB6** (Pin 14) | `MOTOR_FL_PWM` | `TIM4_CH1` (AF2) @ 20 kHz | Cytron 1: `PWM1` | Front-Left Speed Control |
| **PB4** (Pin 12) | `MOTOR_FL_DIR` | `GPIO_Output` (Push-Pull) | Cytron 1: `DIR1` | Front-Left Direction (HIGH=Fwd, LOW=Rev) |
| **PB7** (Pin 15) | `MOTOR_RL_PWM` | `TIM4_CH2` (AF2) @ 20 kHz | Cytron 1: `PWM2` | Rear-Left Speed Control |
| **PB5** (Pin 13) | `MOTOR_RL_DIR` | `GPIO_Output` (Push-Pull) | Cytron 1: `DIR2` | Rear-Left Direction (HIGH=Fwd, LOW=Rev) |
| **PB8** (Pin 16) | `MOTOR_FR_PWM` | `TIM4_CH3` (AF2) @ 20 kHz | Cytron 2: `PWM1` | Front-Right Speed Control |
| **PC13** (Pin 39)| `MOTOR_FR_DIR` | `GPIO_Output` (Push-Pull) | Cytron 2: `DIR1` | Front-Right Direction (Inverted in software) |
| **PB9** (Pin 17) | `MOTOR_RR_PWM` | `TIM4_CH4` (AF2) @ 20 kHz | Cytron 2: `PWM2` | Rear-Right Speed Control |
| **PC14** (Pin 38)| `MOTOR_RR_DIR` | `GPIO_Output` (Push-Pull) | Cytron 2: `DIR2` | Rear-Right Direction (Inverted in software) |
| **GND** (Pin 19) | `GND` | Ground Reference | Cytron 1 & 2: `GND` | Common Logic Ground |

*Electrical Note:* Connect a 12V–24V main LiPo battery directly to the `POWER +` and `POWER -` screw terminals of both Cytron drivers. Never power high-current motors through the BlackPill power rails.

---

### 2.3. Inertial Measurement Unit (MPU-6050 6-DOF IMU)

The MPU-6050 6-axis gyroscope and accelerometer provides yaw rate and tilt telemetry over `I2C2` at 400 kHz Fast Mode:

| BlackPill Pin | Net Name | Mode / Alternate Function | MPU-6050 Pin | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **PB10** (Pin 24) | `IMU_I2C2_SCL` | `I2C2_SCL` (AF4) Open-Drain | `SCL` | Fast Mode 400 kHz, $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **PB3** (Pin 11)  | `IMU_I2C2_SDA` | `I2C2_SDA` (AF9) Open-Drain | `SDA` | Fast Mode 400 kHz, $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **3V3** (Pin 20)  | `+3.3V`        | Regulated Power Out | `VCC` | Supplies 3.3V logic rail |
| **GND** (Pin 19)  | `GND`          | Ground Reference | `GND` | Ground return |
| —                 | —              | Hardwired to GND | `AD0` | Selects 7-bit I2C Address `0x68` |

---

### 2.4. Global Positioning System (u-blox NEO-6M GPS)

The u-blox NEO-6M module provides GPS coordinates via NMEA `$GPGGA` sentences:

| BlackPill Pin | Net Name | Mode / Alternate Function | NEO-6M Pin | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **PA2** (Pin 33) | `GPS_UART2_TX` | `USART2_TX` (AF7) | `RX` | Optional configuration uplink (9600 Baud) |
| **PA3** (Pin 32) | `GPS_UART2_RX` | `USART2_RX` (AF7) | `TX` | NMEA sentence stream into `DMA1_Stream5` |
| **5V** (Pin 18)   | `+5.0V`        | Board Power Rail | `VCC` | Powers onboard 3.3V LDO on GPS module |
| **GND** (Pin 19)  | `GND`          | Ground Reference | `GND` | Common ground return |

---

### 2.5. Diagnostic Indicators Subsystem (Buzzer & 3 Status LEDs)

`STM32_Mobility` integrates dedicated visual and acoustic diagnostic hardware on pins `PB12` through `PB15`:

| BlackPill Pin | Signal Net | Output Type | Hardware Indicator | Functional State & Meaning |
| :---: | :---: | :---: | :---: | :--- |
| **PB12** (Pin 1) | `MOBILITY_BUZZER` | Push-Pull | Active Piezo Buzzer | Sounds continuously on watchdog trip or emergency mine alert. |
| **PB13** (Pin 2) | `MOBILITY_LED1`   | Push-Pull | Red LED (Alarm) | **Watchdog / Error Alert:** Lit when `/cmd_vel` is lost for $> 200\,\text{ms}$ or mine trigger is asserted. |
| **PB14** (Pin 3) | `MOBILITY_LED2`   | Push-Pull | Yellow LED (Motion) | **Locomotion Active:** Lit when wheel duty cycle exceeds deadband threshold ($> 8\%$). |
| **PB15** (Pin 4) | `MOBILITY_LED3`   | Push-Pull | Green LED (Ready) | **System / ROS OK:** Lit solid when micro-ROS client successfully links to the host agent. |

*Wiring Note:* Use $330\,\Omega$ current-limiting series resistors with LEDs `PB13`, `PB14`, `PB15` to ground. Connect `PB12` to a 2N2222 NPN transistor or logic-level MOSFET gate to switch the active buzzer.

---

### 2.6. micro-ROS Host Link & ST-Link SWD Programming

#### micro-ROS Serial Interconnect:
Connect an FTDI / CP2102 USB-to-UART adapter between the host SBC (Raspberry Pi 5) and the BlackPill:
| BlackPill Pin | Net Name | Alternate Function | USB-UART Adapter Pin | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **PA9** (Pin 6)   | `MICROROS_TX` | `USART1_TX` (AF7) | `RXD` | Transmit to host (`DMA2_Stream7`) |
| **PA10** (Pin 7)  | `MICROROS_RX` | `USART1_RX` (AF7) | `TXD` | Receive from host (`DMA2_Stream2` Circular) |
| **GND** (Pin 19)  | `GND`         | Ground Reference  | `GND` | Common signal ground |

#### ST-Link V2 SWD Programming Header:
Connect ST-Link programmer to the 4-pin SWD debug header at the bottom of the BlackPill:
| Header Pin | Signal Name | MCU Pad | ST-Link Pin |
| :---: | :---: | :---: | :---: |
| **3V3** | Target Voltage | `3V3` | `3.3V (Pin 1)` |
| **SWDIO** | SWD Data | `PA13` | `SWDIO (Pin 2)` |
| **SWCLK** | SWD Clock | `PA14` | `SWCLK (Pin 4)` |
| **GND** | Ground | `GND` | `GND (Pin 6)` |

---

### 2.7. Critical Pin Multiplexer Conflict Resolution (TIM4 vs I2C1)

On the STM32F411 microcontroller, `I2C1` default pins (`PB6` SCL, `PB7` SDA) and remapped pins (`PB8` SCL, `PB9` SDA) completely overlap the 4 channels of `TIM4` (`PB6`, `PB7`, `PB8`, `PB9`).  
Selecting `TIM4` for 4-channel motor PWM starves `I2C1` of physical pins.

**The Solution:**
`STM32_Mobility` migrates the MPU-6050 IMU to **`I2C2`**:
- `I2C2_SCL`: Routed to **`PB10`** (Pin 24, AF4).
- `I2C2_SDA`: Routed to **`PB3`** (Pin 11, AF9).

This guarantees 100% independent hardware execution without pin multiplexing conflicts.

---

## 3. micro-ROS Communication Contract

### 3.1. Topic Routing & Interface Specifications

`STM32_Mobility` communicates natively with ROS 2 Humble using standard ROS 2 message definitions:

| Topic Name | ROS 2 Message Type | Direction | Target Rate | Description |
| :--- | :--- | :---: | :---: | :--- |
| **`/cmd_vel`** | `geometry_msgs/msg/Twist` | **Subscribed** | Up to 20 Hz | Target linear translational velocity ($v_x$) and angular rotational velocity ($\omega_z$). Feeds watchdog timer. |
| **`/imu/data`** | `sensor_msgs/msg/Imu` | **Published** | 50 Hz (20 ms) | 6-DOF IMU orientation quaternion, angular velocity ($\text{rad/s}$), and linear acceleration ($\text{m/s}^2$). |
| **`/gps/fix`** | `sensor_msgs/msg/NavSatFix` | **Published** | 5 Hz (200 ms) | Latitude, longitude, altitude, and fix status decoded from NMEA `$GPGGA`. |
| **`/stm32_heartbeat`** | `std_msgs/msg/Bool` | **Published** | 1 Hz (1000 ms)| Periodic boolean liveness heartbeat confirming ECU health. |

---

### 3.2. Configurable Baud Rate Serial Transport Layer

The micro-ROS transport is exposed via `#define MICROROS_BAUDRATE` in `Core/Inc/mobility_pin_config.h`:
```c
#ifndef MICROROS_BAUDRATE
#define MICROROS_BAUDRATE (921600U) /* Supported: 115200, 460800, 921600 */
#endif
```

#### DMA Implementation Details:
- **Circular Reception (`DMA2_Stream2`)**: Continuous circular ring-buffer reception prevents buffer overrun during high-frequency topic bursts.
- **Direct Memory Access Transmission (`DMA2_Stream7`)**: Non-blocking transmit staging buffer streams packets at up to 92.16 KB/s without stalling FreeRTOS tasks.

---

## 4. Task Execution Flow & Safety Architecture

### 4.1. Rate Monotonic Scheduling (RMS) Hierarchy

The firmware implements four CMSIS_V2 tasks partitioned by real-time criticality:

| Task Name | Priority | Stack Size | Period / Rate | Primary Functions |
| :--- | :--- | :---: | :---: | :--- |
| **`Task_SafetyWDT`** | `osPriorityRealtime` | 512 Words (2 KB) | 10 ms (100 Hz) | Evaluates `/cmd_vel` timeout; executes emergency mine cutoff; drives PB12 buzzer & PB13 red LED. |
| **`Task_MotorCtrl`** | `osPriorityHigh` | 512 Words (2 KB) | 20 ms (50 Hz) | Skid-steer kinematics; 8% deadband compensation; yaw gyro damping; TIM4 CCR duty cycle updates. |
| **`Task_SensorAcq`** | `osPriorityAboveNormal` | 512 Words (2 KB) | 20 ms (50 Hz) | Drains USART2 GPS DMA buffer; samples MPU-6050 6-DOF IMU over I2C2 Fast Mode. |
| **`Task_MicroROS`** | `osPriorityNormal` | 1024 Words (4 KB) | 5 ms spin | Spins micro-ROS executor; serializes `/imu/data`, `/gps/fix`, `/stm32_heartbeat`; monitors link. |

---

### 4.2. 200 ms Safety Watchdog Cutoff & Mine Trigger Interlock

Safety is enforced by `Task_SafetyWatchdog` running at `osPriorityRealtime`:
1. **Heartbeat Monitoring**: Every received `/cmd_vel` message calls `Task_SafetyWatchdog_Feed()`, resetting the tick counter.
2. **200 ms Command Timeout**: If the elapsed duration since the last feed exceeds $200\,\text{ms}$, the watchdog trips:
   - All 4 `TIM4` PWM channels immediately clamp to `CCR = 0` (0% duty cycle).
   - Red LED (`PB13`) illuminates solid.
   - Audible buzzer (`PB12`) activates.
3. **Emergency Mine Interlock**: When the auxiliary payload ECU or central coverage planner detects a surface or buried landmine, `Task_SafetyWatchdog_TriggerMine()` immediately forces an emergency stop, preventing the rover wheels from traversing over the detected threat.

---

### 4.3. Skid-Steer Kinematics, Polarity & Deadband Compensation

#### Inverse Kinematics:
Differential skid-steer kinematics maps linear velocity $v_x$ and angular velocity $\omega_z$ to left and right wheel velocities:
$$v_{\text{left}} = v_x - \left(\frac{L}{2}\right) \omega_z, \quad v_{\text{right}} = v_x + \left(\frac{L}{2}\right) \omega_z$$
where track gauge $L = 0.55\,\text{m}$.

#### Right-Side Polarity Inversion:
Because motors on opposite sides of the chassis face opposite directions, right-side direction pins (`PC13` and `PC14`) are inverted in software so that positive commanded velocities rotate all four wheels forward symmetrically.

#### 8% Friction Deadband Compensation:
To overcome initial gearbox stiction and loose sand static resistance, normalized duty cycles jump non-linearly over an 8% deadband threshold:
$$\text{duty}_{\text{comp}} = \operatorname{sgn}(\text{duty}) \times \left(0.08 + 0.92 \times |\text{duty}|\right)$$
When target velocity is exactly zero ($|v| \le 0.001$), duty cycle outputs exactly $0\%$.

---

### 4.4. Closed-Loop Gyro Yaw Damping in Loose Terrain

When traversing desert terrain, asymmetrical wheel slippage can cause unintentional heading deflection. When the rover is commanded to drive straight ($|\omega_z| < 0.05\,\text{rad/s}$ and $|v_x| > 0.05\,\text{m/s}$), the motor control task injects closed-loop gyro yaw rate damping:
$$\omega_{z,\text{effective}} = \omega_{z,\text{target}} - 0.15 \times \omega_{z,\text{gyro}}$$
clamped to $\pm 0.4\,\text{rad/s}$. This automatically compensates for sand drift and maintains straight tracks.

---

## 5. Build, Flash & Host Verification

### 5.1. STM32CubeIDE Build Instructions

1. Open **STM32CubeIDE** (Version 1.14.0 or newer).
2. Select **File** $\to$ **Import...** $\to$ **General** $\to$ **Existing Projects into Workspace**.
3. Browse to `e:/MineSweeper/STM32_Mobility` and click **Finish**.
4. Confirm project properties include:
   - **Compiler Include Path**: Include directories for micro-ROS, FreeRTOS, and BSP drivers.
   - **Linker Libraries**: `-lmicroros` linked against `libmicroros.a`.
   - **Floating-Point ABI**: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`.
5. Select **Project** $\to$ **Build Project** (or press `Ctrl+B`). Confirm 0 Errors and 0 Warnings.

---

### 5.2. ST-Link SWD Flashing

Using **STM32CubeProgrammer CLI**:
```powershell
STM32_Programmer_CLI -c port=SWD mode=UR -w "e:\MineSweeper\STM32_Mobility\Debug\STM32_Mobility.elf" -v -rst
```
Or via STM32CubeIDE: Click **Run** $\to$ **Run** (`Ctrl+F11`) with ST-Link GDB server selected.

---

### 5.3. micro-ROS Agent Host Execution

On the ROS 2 Humble host computer (Raspberry Pi 5 or Linux workstation):
```bash
# Launch micro-ROS serial agent at 921600 baud
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

Verify connection:
```bash
# Check active topics
ros2 topic list

# Echo navigation sensors
ros2 topic echo /imu/data
ros2 topic echo /gps/fix

# Send test drive command (0.3 m/s forward)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

---

### 5.4. Automated Test Suite Execution (pytest)

The firmware implementation is validated by a 61-test Python suite in `tests/`:

```powershell
# Run the documentation validation test
pytest tests/tier1_features/test_r4_cubeguide_doc.py -v

# Run the complete test suite (all 61 tests)
pytest tests -v
```

Expected output:
```text
============================= 61 passed in 0.16s ==============================
```

---

## 6. Repository Structure & Code Layout

```
STM32_Mobility/
├── CUBEMX_CONFIGURATION_GUIDE.md    # Dedicated CubeMX GUI & clock setup guide
├── README.md                        # Comprehensive system documentation
├── STM32_Mobility.ioc               # STM32CubeMX hardware definition project
├── Core/
│   ├── Inc/
│   │   ├── main.h                   # Main HAL prototypes
│   │   ├── mobility_pin_config.h    # Centralized hardware pin & peripheral macros
│   │   ├── FreeRTOSConfig.h         # FreeRTOS 64 KB heap & CMSIS_V2 settings
│   │   └── stm32f4xx_hal_conf.h     # HAL peripheral driver enables
│   ├── BSP/
│   │   ├── cytron_mdd10a_driver.c/.h       # 4-motor PWM & DIR driver
│   │   ├── mobility_diagnostics_driver.c/.h# PB12 buzzer & PB13-PB15 LEDs driver
│   │   ├── mpu6050_driver.c/.h             # 6-DOF IMU driver (I2C2 Fast Mode)
│   │   └── neo6m_driver.c/.h               # u-blox GPS NMEA parser (USART2 DMA)
│   ├── Middleware/
│   │   └── microros/
│   │       ├── microros_client.c/.h        # micro-ROS client state machine & topics
│   │       ├── microros_msgs.h             # ROS 2 message struct definitions
│   │       └── microros_transport.c/.h     # USART1 DMA circular transport layer
│   ├── Tasks/
│   │   ├── task_safety_watchdog.c/.h       # 200 ms safety watchdog task (Realtime)
│   │   ├── task_motor_control.c/.h         # Skid-steer kinematics & deadband task (High)
│   │   ├── task_sensor_acq.c/.h            # IMU & GPS sensor acquisition task (AboveNormal)
│   │   └── task_microros.c/.h              # micro-ROS executor spin task (Normal)
│   └── Src/
│       ├── main.c                   # System initialization & FreeRTOS task spawns
│       ├── freertos.c               # FreeRTOS hook implementations
│       └── stm32f4xx_it.c           # Hardware ISR vectors (USART1, USART2, DMA)
└── Middlewares/
    └── Third_Party/
        ├── FreeRTOS/                # FreeRTOS kernel source tree (heap_4)
        └── micro_ros_stm32cubemx_utils/ # micro-ROS static library & headers
```

---

*Authored for the Autonomous Minesweepers 2026 Competition.*
