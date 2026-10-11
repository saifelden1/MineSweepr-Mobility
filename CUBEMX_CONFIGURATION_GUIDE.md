# STM32CubeMX Graphical Configuration Guide: STM32_Mobility ECU
## Complete Step-by-Step Hardware & Middleware Setup for Autonomous Minesweeper Locomotion & Navigation

**Target MCU:** WeAct Studio STM32F411CEU6 BlackPill (ARM Cortex-M4F @ 96 MHz)  
**RTOS & Middleware:** FreeRTOS (CMSIS_V2) + micro-ROS Client over UART DMA @ 921600 Baud  
**Project:** `STM32_Mobility` (Autonomous Minesweeper Drivetrain & Sensor Platform)  
**Revision:** 2.1.0 (Production Release)  

---

## Table of Contents

1. [Executive Overview](#1-executive-overview)
2. [Physical 40-Pin BlackPill Pinout & Hardware Allocation](#2-physical-40-pin-blackpill-pinout--hardware-allocation)
   - [2.1. Physical 40-Pin BlackPill ASCII Reference Diagram](#21-physical-40-pin-blackpill-ascii-reference-diagram)
   - [2.2. Complete Pinout Allocation & Alternate Function Table](#22-complete-pinout-allocation--alternate-function-table)
   - [2.3. Critical Hardware Conflict Analysis: TIM4 (PB6–PB9) vs I2C1 Collision & I2C2 Resolution](#23-critical-hardware-conflict-analysis-tim4-pb6pb9-vs-i2c1-collision--i2c2-resolution)
3. [Clock Tree Configuration (25.0 MHz HSE → 96.0 MHz SYSCLK)](#3-clock-tree-configuration-250-mhz-hse--960-mhz-sysclk)
   - [3.1. Mathematical Derivation & PLL Constraints](#31-mathematical-derivation--pll-constraints)
   - [3.2. Visual Clock Tree Flow Diagram](#32-visual-clock-tree-flow-diagram)
   - [3.3. CubeMX Clock Configuration Tab Settings](#33-cubemx-clock-configuration-tab-settings)
4. [Step-by-Step CubeMX Peripherals Configuration](#4-step-by-step-cubemx-peripherals-configuration)
   - [4.1. System Core & HAL Timebase (SYS: TIM11)](#41-system-core--hal-timebase-sys-tim11)
   - [4.2. TIM4 PWM Generation: 20 kHz 4-Motor Carrier](#42-tim4-pwm-generation-20-khz-4-motor-carrier)
   - [4.3. I2C2 Fast Mode Setup: MPU-6050 6-DOF IMU (400 kHz)](#43-i2c2-fast-mode-setup-mpu-6050-6-dof-imu-400-khz)
   - [4.4. USART1 DMA Setup: High-Speed micro-ROS Serial Link (921600 Baud)](#44-usart1-dma-setup-high-speed-micro-ros-serial-link-921600-baud)
   - [4.5. USART2 DMA Setup: u-blox NEO-6M GPS Receiver (9600 Baud)](#45-usart2-dma-setup-u-blox-neo-6m-gps-receiver-9600-baud)
   - [4.6. GPIO Digital Outputs: Drivetrain Direction & Diagnostic Subsystem](#46-gpio-digital-outputs-drivetrain-direction--diagnostic-subsystem)
5. [FreeRTOS (CMSIS_V2) Operating System Configuration](#5-freertos-cmsis_v2-operating-system-configuration)
   - [5.1. Kernel & Memory Parameter Configuration](#51-kernel--memory-parameter-configuration)
   - [5.2. Rate Monotonic Task Allocation Table](#52-rate-monotonic-task-allocation-table)
6. [STM32CubeIDE Build & micro-ROS Static Library Integration](#6-stm32cubeide-build--micro-ros-static-library-integration)
   - [6.1. Compiler Include Directories (-I)](#61-compiler-include-directories--i)
   - [6.2. Linker Search Directories (-L) & Static Library (-lmicroros)](#62-linker-search-directories--l--static-library--lmicroros)
   - [6.3. Hardware Floating-Point Unit (FPU) Toolchain Flags](#63-hardware-floating-point-unit-fpu-toolchain-flags)
   - [6.4. Code Generation Settings in CubeMX Project Manager](#64-code-generation-settings-in-cubemx-project-manager)
7. [Hardware Verification & Live Validation Procedures](#7-hardware-verification--live-validation-procedures)

---

## 1. Executive Overview

The `STM32_Mobility` ECU is the primary locomotion and navigation microcontroller for the Autonomous Minesweeper Rover. Hosted on an ultra-compact **WeAct Studio STM32F411CEU6 BlackPill**, the firmware performs:
- **4-Motor Locomotion**: 4 independent PWM channels on `TIM4` running at an ultrasonic 20 kHz carrier frequency to drive dual Cytron MDD10A dual-channel H-bridges, accompanied by 4 independent direction lines.
- **Closed-Loop Heading Damping**: 6-DOF inertial sensing via an MPU-6050 accelerometer/gyroscope over `I2C2` Fast Mode (400 kHz), stabilizing rover trajectories across loose sand.
- **Georeferencing**: Non-blocking NMEA sentence reception from a u-blox NEO-6M GPS module over `USART2` DMA circular ring buffer at 9600 baud.
- **Safety Interlock**: Hardware-priority FreeRTOS watchdog task enforcing a strict < 200 ms timeout cutoff, halting all motor PWM lines if command teleop ceases or mine proximity is asserted.
- **Diagnostics**: Comprehensive board status indication through an audible buzzer (`PB12`) and three status LEDs (`PB13` Red: Watchdog Trip, `PB14` Yellow: Motion Active, `PB15` Green: ROS 2 System Ready).
- **micro-ROS Serial Bridge**: Real-time ROS 2 communication over `USART1` DMA at 921600 baud, exchanging `/cmd_vel`, `/imu/data`, `/gps/fix`, and `/stm32_heartbeat`.

This document specifies the exact graphical configuration required in STM32CubeMX to recreate or modify this firmware from scratch.

---

## 2. Physical 40-Pin BlackPill Pinout & Hardware Allocation

### 2.1. Physical 40-Pin BlackPill ASCII Reference Diagram

The WeAct Studio STM32F411CEU6 BlackPill module exposes 40 dual-inline edge pins, a USB Type-C connector at the top edge, and a 4-pin SWD programming header at the bottom edge.

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

### 2.2. Complete Pinout Allocation & Alternate Function Table

All peripheral pins are declared centrally in `Core/Inc/mobility_pin_config.h`. CubeMX must be configured to match these assignments precisely:

| Physical Pin # | Pin Label | Alternate Function (AF) | CubeMX Mode | Signal Net Name | Peripheral / Target Device | Electrical & Functional Notes |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **14** | **PB6** | AF2 | `TIM4_CH1` | `MOTOR_FL_PWM` | Cytron 1: Front-Left PWM | 20 kHz PWM Carrier (PSC=0, ARR=4799) |
| **15** | **PB7** | AF2 | `TIM4_CH2` | `MOTOR_RL_PWM` | Cytron 1: Rear-Left PWM | 20 kHz PWM Carrier (PSC=0, ARR=4799) |
| **16** | **PB8** | AF2 | `TIM4_CH3` | `MOTOR_FR_PWM` | Cytron 2: Front-Right PWM | 20 kHz PWM Carrier (PSC=0, ARR=4799) |
| **17** | **PB9** | AF2 | `TIM4_CH4` | `MOTOR_RR_PWM` | Cytron 2: Rear-Right PWM | 20 kHz PWM Carrier (PSC=0, ARR=4799) |
| **12** | **PB4** | GPIO | `GPIO_Output` | `MOTOR_FL_DIR` | Cytron 1: Front-Left DIR | Logic HIGH = Fwd, LOW = Rev |
| **13** | **PB5** | GPIO | `GPIO_Output` | `MOTOR_RL_DIR` | Cytron 1: Rear-Left DIR | Logic HIGH = Fwd, LOW = Rev |
| **39** | **PC13**| GPIO | `GPIO_Output` | `MOTOR_FR_DIR` | Cytron 2: Front-Right DIR | Inverted in software for symmetrical drive |
| **38** | **PC14**| GPIO | `GPIO_Output` | `MOTOR_RR_DIR` | Cytron 2: Rear-Right DIR | Inverted in software for symmetrical drive |
| **24** | **PB10**| AF4 | `I2C2_SCL` | `IMU_I2C2_SCL` | MPU-6050 SCL (400 kHz) | Open-drain, $4.7\,\text{k}\Omega$ pull-up to 3.3V (**Resolves TIM4/I2C1 collision**) |
| **11** | **PB3** | AF9 | `I2C2_SDA` | `IMU_I2C2_SDA` | MPU-6050 SDA (400 kHz) | Open-drain, $4.7\,\text{k}\Omega$ pull-up to 3.3V (**Resolves TIM4/I2C1 collision**) |
| **33** | **PA2** | AF7 | `USART2_TX` | `GPS_UART2_TX` | u-blox NEO-6M Serial TX | 9600 Baud, 8N1 |
| **32** | **PA3** | AF7 | `USART2_RX` | `GPS_UART2_RX` | u-blox NEO-6M Serial RX | DMA1 Stream 5 Circular ring buffer |
| **6**  | **PA9** | AF7 | `USART1_TX` | `MICROROS_TX` | micro-ROS Serial TX | 921600 Baud, DMA2 Stream 7 Normal |
| **7**  | **PA10**| AF7 | `USART1_RX` | `MICROROS_RX` | micro-ROS Serial RX | 921600 Baud, DMA2 Stream 2 Circular ring |
| **1**  | **PB12**| GPIO | `GPIO_Output` | `MOBILITY_BUZZER` | Diagnostic Audible Buzzer | Active alarm transistor / buzzer drive |
| **2**  | **PB13**| GPIO | `GPIO_Output` | `MOBILITY_LED1` | Diagnostic LED 1 (Red) | Watchdog trip & emergency interlock alarm |
| **3**  | **PB14**| GPIO | `GPIO_Output` | `MOBILITY_LED2` | Diagnostic LED 2 (Yellow)| Locomotion active indicator ($|\text{cmd}| > \text{deadband}$) |
| **4**  | **PB15**| GPIO | `GPIO_Output` | `MOBILITY_LED3` | Diagnostic LED 3 (Green) | System initialized & micro-ROS client active |
| **Header** | **PA13** | AF0 | `SYS_JTMS-SWDIO` | `SWDIO` | ST-Link SWD Debug Data | Serial Wire Debug bidirectional data |
| **Header** | **PA14** | AF0 | `SYS_JTCK-SWCLK` | `SWCLK` | ST-Link SWD Debug Clock| Serial Wire Debug clock |
| **19, 22** | **GND**  | Power| `GND` | `GND` | Common Ground Plane | Connect to battery return & motor driver GND |
| **20, 23** | **3V3**  | Power| `3V3` | `+3.3V` | Regulated Logic Output | Supplies MPU-6050 logic rail |
| **18, 21** | **5V**   | Power| `5V` | `+5.0V` | Board Power Input / VBUS | Direct 5V rail from DC-DC buck converter |

---

### 2.3. Critical Hardware Conflict Analysis: TIM4 (PB6–PB9) vs I2C1 Collision & I2C2 Resolution

An architectural challenge arises when attempting to configure four independent timer PWM channels alongside standard I2C buses on the STM32F411 microcontroller.

#### The Root Hardware Contention
1. **4-Motor Drive Requirements**: Symmetrical 4-wheel skid-steer propulsion requires 4 discrete hardware PWM outputs. On the STM32F411CEU6 package, the only general-purpose timer possessing four channels on standard GPIO pins without consuming SWD debug lines or primary communication buses is **TIM4**:
   - `PB6`: `TIM4_CH1` (Front-Left PWM)
   - `PB7`: `TIM4_CH2` (Rear-Left PWM)
   - `PB8`: `TIM4_CH3` (Front-Right PWM)
   - `PB9`: `TIM4_CH4` (Rear-Right PWM)
2. **I2C1 Default Pin Mappings**: STM32F411 routes the primary `I2C1` peripheral exclusively to:
   - Primary mapping: `PB6` (SCL) and `PB7` (SDA)
   - Remapped alternate: `PB8` (SCL) and `PB9` (SDA)
3. **The Multiplexer Deadlock**:
   Every available pin for `I2C1` is occupied by `TIM4_CH1..CH4`. The internal GPIO alternate function multiplexer register (`AFRL`/`AFRH`) allows only one peripheral interconnect per physical pad at any instant. Attempting to assign `I2C1` to `PB6`/`PB7` or `PB8`/`PB9` strips `TIM4` of two PWM channels, crippling the 4-wheel drive system.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                   STM32F411CEU6 PIN MULTIPLEXER DEADLOCK                    │
│                                                                             │
│  Physical Pad PB6 ───► [AF2: TIM4_CH1 (PWM FL)]  OR  [AF4: I2C1_SCL]       │
│  Physical Pad PB7 ───► [AF2: TIM4_CH2 (PWM RL)]  OR  [AF4: I2C1_SDA]       │
│  Physical Pad PB8 ───► [AF2: TIM4_CH3 (PWM FR)]  OR  [AF4: I2C1_SCL]       │
│  Physical Pad PB9 ───► [AF2: TIM4_CH4 (PWM RR)]  OR  [AF4: I2C1_SDA]       │
│                                                                             │
│  CONCLUSION: Assigning TIM4 to all 4 motors starves I2C1 completely!        │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### The Architectural Resolution: I2C2 Remapping
To achieve conflict-free operation, the firmware bypasses `I2C1` entirely and provisions **`I2C2`**:
- **`I2C2_SCL`**: Routed to **`PB10`** (Pin 24) via Alternate Function `AF4`.
- **`I2C2_SDA`**: Routed to **`PB3`** (Pin 11) via Alternate Function `AF9`.

**System Advantages:**
1. Zero pin contention: `TIM4` maintains full four-channel PWM generation on `PB6`, `PB7`, `PB8`, and `PB9`.
2. High-speed sensor bandwidth: `I2C2` runs independently at 400 kHz Fast Mode, sampling 6-DOF IMU registers at 50 Hz with zero jitter.
3. Centralized pin declarations: `mobility_pin_config.h` sets `IMU_I2C_INSTANCE` to `I2C2`, matching the CubeMX mapping.

---

## 3. Clock Tree Configuration (25.0 MHz HSE → 96.0 MHz SYSCLK)

The WeAct Studio BlackPill integrates an onboard **25.000 MHz High-Speed External (HSE)** crystal. To reach maximum processing throughput for floating-point kinematics, Kalman filtering, and micro-ROS message serialization, the Phase-Locked Loop (PLL) is configured for the microcontroller's maximum rated frequency of **96.0 MHz**.

### 3.1. Mathematical Derivation & PLL Constraints

The STM32F411 main PLL produces the system clock according to:
$$f_{\text{VCO\_IN}} = \frac{f_{\text{HSE}}}{M}$$
$$f_{\text{VCO\_OUT}} = f_{\text{VCO\_IN}} \times N = \left(\frac{f_{\text{HSE}}}{M}\right) \times N$$
$$f_{\text{SYSCLK}} = \frac{f_{\text{VCO\_OUT}}}{P} = \frac{\left(\frac{f_{\text{HSE}}}{M}\right) \times N}{P}$$
$$f_{\text{PLL48CK}} = \frac{f_{\text{VCO\_OUT}}}{Q} = \frac{\left(\frac{f_{\text{HSE}}}{M}\right) \times N}{Q}$$

#### Parameter Selection:
1. **$M = 25$**:
   $$f_{\text{VCO\_IN}} = \frac{25.0\,\text{MHz}}{25} = 1.0\,\text{MHz}$$
   *(Hardware constraint: $1.0\,\text{MHz} \le f_{\text{VCO\_IN}} \le 2.0\,\text{MHz}$ — Compliant)*
2. **$N = 192$**:
   $$f_{\text{VCO\_OUT}} = 1.0\,\text{MHz} \times 192 = 192.0\,\text{MHz}$$
   *(Hardware constraint: $100.0\,\text{MHz} \le f_{\text{VCO\_OUT}} \le 432.0\,\text{MHz}$ — Compliant)*
3. **$P = 2$**:
   $$f_{\text{SYSCLK}} = \frac{192.0\,\text{MHz}}{2} = 96.0\,\text{MHz}$$
   *(Hardware constraint: $f_{\text{SYSCLK}} \le 96.0\,\text{MHz}$ — Exact Maximum Rated Speed)*
4. **$Q = 4$**:
   $$f_{\text{PLL48CK}} = \frac{192.0\,\text{MHz}}{4} = 48.0\,\text{MHz}$$
   *(Hardware constraint: USB OTG FS and SDIO require exact $48.0\,\text{MHz}$ — Compliant)*

#### Bus Prescalers & Flash Wait States:
- **AHB Clock ($HCLK$)**: Prescaler = `/1` $\implies 96.0\,\text{MHz}$.
- **APB1 Low-Speed Peripheral Bus ($PCLK1$)**: Prescaler = `/2` $\implies 48.0\,\text{MHz}$ (Maximum rated APB1 clock is $50\,\text{MHz}$).
  - *Timer Clock APB1*: When APB1 prescaler $\ne 1$, hardware doubles the timer clock: $48.0\,\text{MHz} \times 2 = \mathbf{96.0\,\text{MHz}}$ (Powers `TIM4`, `TIM2`, `TIM3`).
- **APB2 High-Speed Peripheral Bus ($PCLK2$)**: Prescaler = `/1` $\implies 96.0\,\text{MHz}$ (Maximum rated APB2 clock is $100\,\text{MHz}$).
  - *Timer Clock APB2*: Timer multiplier is $\times 1 \implies \mathbf{96.0\,\text{MHz}}$ (Powers `TIM1`, `TIM9`, `TIM10`, `TIM11`).
- **Flash Latency**: At $96.0\,\text{MHz}$ and $V_{DD} = 3.3\,\text{V}$, the embedded Flash memory requires **3 Wait States (`FLASH_LATENCY_3`)**.

---

### 3.2. Visual Clock Tree Flow Diagram

```
[ 25.0 MHz External Crystal (HSE) ]
                 │
                 ▼
          [ /M Divider: /25 ] ───► VCO Input = 1.00 MHz (Valid: 1.0 - 2.0 MHz)
                 │
                 ▼
          [ *N Multiplier: *192 ] ─► VCO Output = 192.00 MHz (Valid: 100 - 432 MHz)
                 │
        ┌────────┴─────────────────────────┐
        │                                  │
 [ /P Divider: /2 ]                [ /Q Divider: /4 ]
        │                                  │
        ▼                                  ▼
 96.00 MHz PLLCLK                  48.00 MHz PLL48CK (USB / SDIO)
        │
        ▼  [ SW Mux: PLLCLK ]
 96.00 MHz SYSCLK
        │
        ▼  [ AHB Prescaler: /1 ]
 96.00 MHz HCLK (Core, Flash, DMA1, DMA2)
        │
        ├──────────────────────────────────┬─────────────────────────────────┐
        ▼                                  ▼                                 ▼
 [ APB1 Prescaler: /2 ]            [ APB2 Prescaler: /1 ]             [ SysTick Timer ]
        │                                  │                          96.00 MHz / 8 or
        ▼                                  ▼                          FreeRTOS Tick
 48.00 MHz PCLK1                   96.00 MHz PCLK2
 (I2C2, USART2)                    (USART1, ADC1)
        │                                  │
        ▼ [ APB1 Timer Multiplier: x2 ]    ▼ [ APB2 Timer Multiplier: x1 ]
 96.00 MHz APB1 Timers (TIM4 Motor PWM)   96.00 MHz APB2 Timers (TIM11 HAL Timebase)
```

---

### 3.3. CubeMX Clock Configuration Tab Settings

Open the **Clock Configuration** tab in STM32CubeMX and apply the following settings:
1. In the **Input Frequency** box next to HSE, enter `25` (MHz).
2. Set **PLL Source Mux** to `HSE`.
3. Set PLL Prescaler **M** to `/25`.
4. Set PLL Multiplier **N** to `*192`.
5. Set PLL Divider **P** to `/2`.
6. Set PLL Divider **Q** to `/4`.
7. Set **System Clock Mux** to `PLLCLK`.
8. Verify **SYSCLK (MHz)** displays `96`.
9. Set **AHB Prescaler** to `/1` (HCLK displays `96 MHz`).
10. Set **APB1 Prescaler** to `/2` (PCLK1 displays `48 MHz`, APB1 Timer Clocks display `96 MHz`).
11. Set **APB2 Prescaler** to `/1` (PCLK2 displays `96 MHz`, APB2 Timer Clocks display `96 MHz`).

---

## 4. Step-by-Step CubeMX Peripherals Configuration

### 4.1. System Core & HAL Timebase (SYS: TIM11)

When running FreeRTOS, the standard Cortex-M `SysTick` timer must be dedicated exclusively to the RTOS scheduler tick (`1 kHz`). The STM32 HAL library must be assigned an independent hardware timer to maintain non-blocking HAL delays (`HAL_GetTick()`) that cannot be preempted or locked by FreeRTOS scheduler suspensions.

1. Under **Pinout & Configuration** $\to$ **System Core** $\to$ **SYS**:
   - **Debug**: `Serial Wire` (Assigns `PA13` as `SWDIO` and `PA14` as `SWCLK`).
   - **Timebase Source**: `TIM11` (Uses hardware Timer 11 for `HAL_GetTick()`).
2. Under **System Core** $\to$ **RCC**:
   - **High Speed Clock (HSE)**: `Crystal/Ceramic Resonator` (Enables onboard 25 MHz oscillator).
   - **Low Speed Clock (LSE)**: `Disable`.

---

### 4.2. TIM4 PWM Generation: 20 kHz 4-Motor Carrier

`TIM4` generates 4 independent, center-aligned/edge-aligned PWM waveforms driving the dual Cytron MDD10A H-bridge drivers at 20 kHz. The 20 kHz frequency is chosen above the human hearing threshold to eliminate audible motor whine while minimizing MOSFET switching losses.

#### Parameter Calculation:
$$f_{\text{PWM}} = \frac{f_{\text{TIM\_CLK}}}{(\text{PSC} + 1) \times (\text{ARR} + 1)}$$
Given $f_{\text{TIM\_CLK}} = 96.0\,\text{MHz}$ (APB1 timer clock):
$$20\,000\,\text{Hz} = \frac{96\,000\,000}{(0 + 1) \times (\text{ARR} + 1)} \implies \text{ARR} + 1 = 4800 \implies \mathbf{\text{ARR} = 4799}$$

#### CubeMX Setup Steps:
1. Under **Timers** $\to$ select **TIM4**.
2. In the Mode panel:
   - **Slave Mode**: `Disable`.
   - **Trigger Source**: `Disable`.
   - **Clock Source**: `Internal Clock`.
   - **Channel 1**: `PWM Generation CH1` (Maps to `PB6`).
   - **Channel 2**: `PWM Generation CH2` (Maps to `PB7`).
   - **Channel 3**: `PWM Generation CH3` (Maps to `PB8`).
   - **Channel 4**: `PWM Generation CH4` (Maps to `PB9`).
3. In **Configuration** $\to$ **Parameter Settings**:
   - **Prescaler (PSC - 16 bits value)**: `0`.
   - **Counter Mode**: `Up`.
   - **Counter Period (AutoReload Register - 16 bits value)**: `4799`.
   - **Internal Clock Division (CKD)**: `No Division`.
   - **auto-reload preload**: `Enable`.
4. In **PWM Generation Channel 1..4** parameters:
   - **Mode**: `PWM mode 1`.
   - **Pulse (16 bits value)**: `0` (Initial 0% duty cycle / stopped).
   - **Output compare preload**: `Enable`.
   - **Fast Mode**: `Disable`.
   - **CH Polarity**: `High`.
5. In **GPIO Settings**:
   - Verify `PB6`, `PB7`, `PB8`, `PB9` have Alternate Function `GPIO_AF2_TIM4`, Output Speed set to `Very High`.

---

### 4.3. I2C2 Fast Mode Setup: MPU-6050 6-DOF IMU (400 kHz)

The MPU-6050 communicates over `I2C2` on `PB10` (SCL) and `PB3` (SDA) to avoid collision with `TIM4`.

1. Under **Connectivity** $\to$ select **I2C2**.
2. In the Mode panel:
   - **I2C**: `I2C`.
3. In the Pinout graphical view:
   - Ensure `PB10` is mapped to `I2C2_SCL`.
   - Ensure `PB3` is mapped to `I2C2_SDA`.
4. In **Parameter Settings**:
   - **I2C Speed Mode**: `Fast Mode`.
   - **I2C Clock Speed (Hz)**: `400000` (400 kHz).
   - **Fast Mode Duty Cycle**: `Duty cycle 16/9` (or `Duty cycle 2`).
   - **Primary Address Length**: `7-bit`.
   - **Dual Address Acknowledged**: `Disable`.
   - **General Call Address Detection**: `Disable`.
   - **Clock No Stretch**: `Disable`.
5. In **GPIO Settings**:
   - Select pins `PB10` and `PB3`.
   - **GPIO Mode**: `Alternate Function Open Drain`.
   - **Pull-up/Pull-down**: `Pull-up` (or external $4.7\,\text{k}\Omega$ resistors).
   - **Maximum Output Speed**: `Very High`.

---

### 4.4. USART1 DMA Setup: High-Speed micro-ROS Serial Link (921600 Baud)

`USART1` transfers micro-ROS telemetry and commands between the STM32 and ROS 2 host. To eliminate CPU intervention and prevent buffer overflows during high-throughput bursts, `USART1` utilizes **Circular DMA Reception** and **Normal DMA Transmission**.

1. Under **Connectivity** $\to$ select **USART1**.
2. In the Mode panel:
   - **Mode**: `Asynchronous`.
   - **Hardware Flow Control (RS232)**: `Disable`.
3. In **Parameter Settings**:
   - **Baud Rate**: `921600` Bits/s (matches `MICROROS_BAUDRATE`).
   - **Word Length**: `8 Bits` (including Parity).
   - **Parity**: `None`.
   - **Stop Bits**: `1`.
   - **Data Direction**: `Receive and Transmit`.
   - **Over Sampling**: `16 Samples`.
4. In **DMA Settings** tab:
   - Click **Add** $\to$ select **USART1_RX**:
     * **Stream**: `DMA2 Stream 2` (Channel 4).
     * **Direction**: `Peripheral To Memory`.
     * **Priority**: `Very High`.
     * **Mode**: `Circular` (Provides continuous ring-buffer reception without dropped packets).
     * **Data Width**: Peripheral = `Byte`, Memory = `Byte`.
   - Click **Add** $\to$ select **USART1_TX**:
     * **Stream**: `DMA2 Stream 7` (Channel 4).
     * **Direction**: `Memory To Peripheral`.
     * **Priority**: `High`.
     * **Mode**: `Normal`.
     * **Data Width**: Peripheral = `Byte`, Memory = `Byte`.
5. In **NVIC Settings** tab:
   - Check **USART1 global interrupt** $\implies$ `Enabled` (Preemption Priority = `5`, Sub Priority = `0`).
   - Check **DMA2 stream 2 global interrupt** $\implies$ `Enabled`.
   - Check **DMA2 stream 7 global interrupt** $\implies$ `Enabled`.

---

### 4.5. USART2 DMA Setup: u-blox NEO-6M GPS Receiver (9600 Baud)

The u-blox NEO-6M GPS module streams NMEA sentences (`$GPGGA`) at 9600 baud. Reception is handled by a circular DMA stream into a ring buffer.

1. Under **Connectivity** $\to$ select **USART2**.
2. In the Mode panel:
   - **Mode**: `Asynchronous`.
3. In **Parameter Settings**:
   - **Baud Rate**: `9600` Bits/s (matches `GPS_UART_BAUDRATE`).
   - **Word Length**: `8 Bits`.
   - **Parity**: `None`.
   - **Stop Bits**: `1`.
4. In **DMA Settings** tab:
   - Click **Add** $\to$ select **USART2_RX**:
     * **Stream**: `DMA1 Stream 5` (Channel 4).
     * **Direction**: `Peripheral To Memory`.
     * **Priority**: `Medium`.
     * **Mode**: `Circular`.
     * **Data Width**: Peripheral = `Byte`, Memory = `Byte`.
5. In **NVIC Settings** tab:
   - Check **USART2 global interrupt** $\implies$ `Enabled`.
   - Check **DMA1 stream 5 global interrupt** $\implies$ `Enabled`.

---

### 4.6. GPIO Digital Outputs: Drivetrain Direction & Diagnostic Subsystem

Configure the following GPIO pins as push-pull digital outputs:

| Pin | User Label | GPIO Mode | Pull | Speed | Functional Role |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **PB4** | `MOTOR_FL_DIR` | Output Push-Pull | No pull | Low | Front-Left Cytron DIR Line |
| **PB5** | `MOTOR_RL_DIR` | Output Push-Pull | No pull | Low | Rear-Left Cytron DIR Line |
| **PC13**| `MOTOR_FR_DIR` | Output Push-Pull | No pull | Low | Front-Right Cytron DIR Line |
| **PC14**| `MOTOR_RR_DIR` | Output Push-Pull | No pull | Low | Rear-Right Cytron DIR Line |
| **PB12**| `MOBILITY_BUZZER` | Output Push-Pull | No pull | Low | Active Alarm Buzzer Drive |
| **PB13**| `MOBILITY_LED1` | Output Push-Pull | No pull | Low | Diagnostic LED 1 (Red: Watchdog Alert) |
| **PB14**| `MOBILITY_LED2` | Output Push-Pull | No pull | Low | Diagnostic LED 2 (Yellow: Locomotion Active) |
| **PB15**| `MOBILITY_LED3` | Output Push-Pull | No pull | Low | Diagnostic LED 3 (Green: ROS 2 System Ready) |

*Configuration Steps in CubeMX:*
1. Left-click each pin on the chip package $\to$ select `GPIO_Output`.
2. Under **System Core** $\to$ **GPIO**, click the pin row and enter the corresponding **User Label** and set initial output level to `Low`.

---

## 5. FreeRTOS (CMSIS_V2) Operating System Configuration

### 5.1. Kernel & Memory Parameter Configuration

1. In the Categories panel $\to$ **Middleware** $\to$ select **FREERTOS**.
2. Set **Interface** to `CMSIS_V2`.
3. In the **Config parameters** tab, configure:
   - **TOTAL_HEAP_SIZE**: `65536` Bytes ($64\,\text{KB}$ allocated via `heap_4.c`).
   - **MEMORY_ALLOCATION**: `Dynamic and Static`.
   - **MINIMAL_STACK_SIZE**: `128` Words ($512\,\text{Bytes}$).
   - **MAX_TASK_NAME_LEN**: `16`.
   - **TICK_RATE_HZ**: `1000` ($1\,\text{ms}$ tick period).
   - **USE_PREEMPTION**: `Enabled`.
   - **USE_TIME_SLICING**: `Enabled`.
   - **USE_PORT_OPTIMISED_TASK_SELECTION**: `Enabled`.
   - **ENABLE_FPU**: `Enabled`.
   - **USE_TASK_FPU_SUPPORT**: `2` (Full hardware FPU context save on every task switch).
   - **CHECK_FOR_STACK_OVERFLOW**: `Option 2`.
   - **LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY**: `5`.

---

### 5.2. Rate Monotonic Task Allocation Table

Under **Tasks and Queues** tab in FREERTOS configuration, create the four operational threads:

| Task Name | Entry Function | Priority (CMSIS_V2) | Stack (Words) | Stack (Bytes) | Period / Frequency | Description & Responsibilities |
| :--- | :--- | :--- | :---: | :---: | :---: | :--- |
| **`Task_SafetyWDT`** | `StartSafetyWatchdogTask` | `osPriorityRealtime` | `512` | 2048 B | 10 ms (100 Hz) | Evaluates `/cmd_vel` heartbeat elapsed ticks. If $> 200\,\text{ms}$, immediately sets all 4 PWM outputs to 0 duty cycle, illuminates Red LED (`PB13`), and sounds buzzer (`PB12`). |
| **`Task_MotorCtrl`** | `StartMotorControlTask` | `osPriorityHigh` | `512` | 2048 B | 20 ms (50 Hz) | Inverse skid-steer kinematics calculation, right-side polarity inversion, 8% deadband compensation, closed-loop gyro heading correction, and TIM4 CCR registers update. |
| **`Task_SensorAcq`** | `StartSensorAcqTask` | `osPriorityAboveNormal` | `512` | 2048 B | 20 ms (50 Hz) | Drains USART2 DMA buffer for NMEA sentence decoding and samples MPU-6050 6-DOF IMU registers over I2C2 Fast Mode. |
| **`Task_MicroROS`** | `StartMicroROSTask` | `osPriorityNormal` | `1024` | 4096 B | 5 ms spin | Manages micro-ROS client state machine, processes incoming `/cmd_vel` callbacks, and serializes `/imu/data` (50 Hz), `/gps/fix` (5 Hz), and `/stm32_heartbeat` (1 Hz). |

---

## 6. STM32CubeIDE Build & micro-ROS Static Library Integration

To compile the firmware with the precompiled micro-ROS static library (`libmicroros.a`), the GCC compiler and linker options must be configured in STM32CubeIDE.

### 6.1. Compiler Include Directories (-I)

In STM32CubeIDE, right-click the project `STM32_Mobility` $\to$ **Properties** $\to$ **C/C++ Build** $\to$ **Settings** $\to$ **Tool Settings** $\to$ **MCU GCC Compiler** $\to$ **Include paths (-I)**. Add the following paths:

```text
../Middlewares/Third_Party/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
../Core/Inc
../Core/Inc/interfaces
../Core/BSP
../Core/Middleware/control
../Core/Middleware/microros
../Core/Tasks
../Middlewares/Third_Party/FreeRTOS/Source/include
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
../Drivers/STM32F4xx_HAL_Driver/Inc
../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
../Drivers/CMSIS/Device/ST/STM32F4xx/Include
../Drivers/CMSIS/Include
```

---

### 6.2. Linker Search Directories (-L) & Static Library (-lmicroros)

In the same Settings window, navigate to **MCU GCC Linker** $\to$ **Libraries**:
1. In **Libraries (-l)**, add:
   ```text
   microros
   ```
   *(Instructs the linker to resolve symbols using `libmicroros.a`).*
2. In **Library search path (-L)**, add:
   ```text
   ../Middlewares/Third_Party/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
   ```

---

### 6.3. Hardware Floating-Point Unit (FPU) Toolchain Flags

Ensure that both the Compiler and Linker utilize the hardware single-precision FPU:
- **Target FPU**: `FPv4-SP-D16` (`-mfpu=fpv4-sp-d16`)
- **Float ABI**: `hard` (`-mfloat-abi=hard`)
- **Instruction Set**: `Thumb-2` (`-mthumb`)
- **Architecture**: `ARMv7E-M` (`-mcpu=cortex-m4`)

*In GCC Command Line Flags:*
```text
-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
```

---

### 6.4. Code Generation Settings in CubeMX Project Manager

In STM32CubeMX, select the **Project Manager** tab:
1. **Project**:
   - **Project Name**: `STM32_Mobility`
   - **Toolchain / IDE**: `STM32CubeIDE`
2. **Code Generator**:
   - Check: `Copy only the necessary library files`.
   - Check: `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`.
   - Check: `Keep User Code when re-generating`.
3. Click **GENERATE CODE** to synthesize the HAL and FreeRTOS scaffolding.

---

## 7. Hardware Verification & Live Validation Procedures

Once the firmware is flashed via ST-Link SWD, execute the following verification steps:

1. **20 kHz Ultrasonic PWM Verification**:
   - Connect an oscilloscope probe to `PB6`, `PB7`, `PB8`, and `PB9`.
   - Measure carrier frequency: Confirm exactly $20.0\,\text{kHz}$ ($\pm 0.1\,\text{kHz}$) with no audible whining.
2. **I2C2 Fast Mode Verification**:
   - Probe `PB10` (SCL) and `PB3` (SDA) with a logic analyzer.
   - Confirm clock period of $2.5\,\mu\text{s}$ ($400\,\text{kHz}$) and successful MPU-6050 WHO_AM_I register read (`0x68`).
3. **Safety Watchdog 200 ms Cutoff Verification**:
   - Publish `/cmd_vel` at 20 Hz: Verify motors spin and Yellow LED (`PB14`) is lit.
   - Terminate `/cmd_vel` publisher: Verify all motor PWM duty cycles clamp to 0 within $< 200\,\text{ms}$, Red LED (`PB13`) illuminates, and Buzzer (`PB12`) activates.
4. **micro-ROS Agent Host Connection**:
   - Launch the micro-ROS agent on the ROS 2 host computer:
     ```bash
     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
     ```
   - Verify agent establishes transport session, and Green LED (`PB15`) lights up solid.
   - Echo topics on host:
     ```bash
     ros2 topic echo /imu/data
     ros2 topic echo /gps/fix
     ros2 topic echo /stm32_heartbeat
     ```
