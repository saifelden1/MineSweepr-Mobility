/**
 * @file mobility_pin_config.h
 * @brief Centralized Hardware Pin & Peripheral Configuration for STM32_Mobility ECU.
 *
 * Microcontroller: STM32F411CEU6 BlackPill (ARM Cortex-M4F @ 96 MHz)
 * System Clock: 96 MHz via 25 MHz HSE PLL (or 16 MHz HSI PLL fallback)
 *
 * Centralizes all pin numbers, GPIO ports, timer channels, and bus handles.
 * Modifying definitions in this header remaps hardware peripherals without
 * changing any driver or application logic.
 */

#ifndef MOBILITY_PIN_CONFIG_H
#define MOBILITY_PIN_CONFIG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/* 1. PHYSICAL ROBOT CONSTANTS & KINEMATICS                                  */
/* ========================================================================= */
#define ROBOT_TRACK_GAUGE_M             (0.55f)   /**< Track width between left/right wheel centers */
#define ROBOT_WHEEL_RADIUS_M            (0.13f)   /**< Outer wheel radius (130 mm) */
#define ROBOT_MAX_LINEAR_VEL_M_S        (1.00f)   /**< Max translational velocity (m/s) */
#define ROBOT_MAX_ANGULAR_VEL_RAD_S     (3.00f)   /**< Max rotational yaw rate (rad/s) */
#define MOTOR_MIN_DEADBAND_DUTY         (0.08f)   /**< Static friction deadband compensation (8%) */
#define MOTOR_PWM_MAX_TICKS             (4799U)   /**< Auto-Reload Register (ARR) for 20 kHz PWM */
#define MOTOR_PWM_PRESCALER             (0U)      /**< Prescaler (PSC) for 96 MHz timer clock */
#define MOTOR_PWM_PERIOD_TICKS          (4799U)   /**< 20 kHz carrier period */
#define WATCHDOG_TIMEOUT_MS             (200U)    /**< Maximum allowed silence on /cmd_vel */

/* ========================================================================= */
/* 2. DRIVETRAIN: 4-MOTOR PWM & DIRECTION PIN MAPPINGS                       */
/* ========================================================================= */
/**
 * Dual Cytron MDD10A Motor Driver Layout:
 * - Driver 1 (Left Drive):
 *     CH1: Front-Left (FL) -> PWM on PB6 (TIM4_CH1), DIR on PB4
 *     CH2: Rear-Left  (RL) -> PWM on PB7 (TIM4_CH2), DIR on PB5
 * - Driver 2 (Right Drive):
 *     CH1: Front-Right (FR) -> PWM on PB8 (TIM4_CH3), DIR on PC13
 *     CH2: Rear-Right  (RR) -> PWM on PB9 (TIM4_CH4), DIR on PC14
 */

/* Front-Left Motor (FL) */
#define MOTOR_FL_PWM_PORT               GPIOB
#define MOTOR_FL_PWM_PIN                GPIO_PIN_6
#define MOTOR_FL_PWM_CHANNEL            TIM_CHANNEL_1
#define MOTOR_FL_DIR_PORT               GPIOB
#define MOTOR_FL_DIR_PIN                GPIO_PIN_4

/* Rear-Left Motor (RL) */
#define MOTOR_RL_PWM_PORT               GPIOB
#define MOTOR_RL_PWM_PIN                GPIO_PIN_7
#define MOTOR_RL_PWM_CHANNEL            TIM_CHANNEL_2
#define MOTOR_RL_DIR_PORT               GPIOB
#define MOTOR_RL_DIR_PIN                GPIO_PIN_5

/* Front-Right Motor (FR) */
#define MOTOR_FR_PWM_PORT               GPIOB
#define MOTOR_FR_PWM_PIN                GPIO_PIN_8
#define MOTOR_FR_PWM_CHANNEL            TIM_CHANNEL_3
#define MOTOR_FR_DIR_PORT               GPIOC
#define MOTOR_FR_DIR_PIN                GPIO_PIN_13

/* Rear-Right Motor (RR) */
#define MOTOR_RR_PWM_PORT               GPIOB
#define MOTOR_RR_PWM_PIN                GPIO_PIN_9
#define MOTOR_RR_PWM_CHANNEL            TIM_CHANNEL_4
#define MOTOR_RR_DIR_PORT               GPIOC
#define MOTOR_RR_DIR_PIN                GPIO_PIN_14

/* Hardware Timer Instance */
#define MOTOR_PWM_TIMER_INSTANCE        TIM4
#define MOTOR_PWM_TIM_CLK_ENABLE()      __HAL_RCC_TIM4_CLK_ENABLE()

/* ========================================================================= */
/* 3. NAVIGATION SENSORS PIN MAPPINGS                                        */
/* ========================================================================= */
/**
 * MPU-6050 6-DOF IMU:
 * Uses I2C2 to resolve pin multiplexing conflict with TIM4 on PB6-PB9.
 * SCL: PB10, SDA: PB3 @ 400 kHz Fast Mode.
 */
#define IMU_I2C_PORT                    I2C2
#define IMU_I2C_INSTANCE                I2C2
#define IMU_I2C_SCL_PORT                GPIOB
#define IMU_I2C_SCL_PIN                 GPIO_PIN_10
#define IMU_I2C_SDA_PORT                GPIOB
#define IMU_I2C_SDA_PIN                 GPIO_PIN_3
#define IMU_I2C_CLK_ENABLE()            __HAL_RCC_I2C2_CLK_ENABLE()
#define IMU_I2C_SPEED_HZ                (400000U)
#define IMU_I2C_ADDRESS                 (0x68U << 1)

/**
 * u-blox NEO-6M GPS Module:
 * Connected over USART2 (PA2 TX / PA3 RX) at 9600 Baud.
 */
#define GPS_UART_PORT                   USART2
#define GPS_UART_INSTANCE               USART2
#define GPS_UART_TX_PORT                GPIOA
#define GPS_UART_TX_PIN                 GPIO_PIN_2
#define GPS_UART_RX_PORT                GPIOA
#define GPS_UART_RX_PIN                 GPIO_PIN_3
#define GPS_UART_BAUDRATE               (9600U)
#define GPS_UART_CLK_ENABLE()           __HAL_RCC_USART2_CLK_ENABLE()

/* ========================================================================= */
/* 4. MICROROS SERIAL TRANSPORT CONFIGURATION                                */
/* ========================================================================= */
/**
 * micro-ROS Client UART Link:
 * Connected over USART1 (PA9 TX / PA10 RX) using Circular DMA.
 */
#define MICROROS_UART_PORT              USART1
#define MICROROS_UART_INSTANCE          USART1
#define MICROROS_UART_TX_PORT           GPIOA
#define MICROROS_UART_TX_PIN            GPIO_PIN_9
#define MICROROS_UART_RX_PORT           GPIOA
#define MICROROS_UART_RX_PIN            GPIO_PIN_10
#define MICROROS_UART_CLK_ENABLE()      __HAL_RCC_USART1_CLK_ENABLE()

#ifndef MICROROS_BAUDRATE
#define MICROROS_BAUDRATE               (921600U) /**< Supported: 115200, 460800, 921600 */
#endif

#ifndef MICROROS_UART_BAUDRATE
#define MICROROS_UART_BAUDRATE          MICROROS_BAUDRATE
#endif

/* ========================================================================= */
/* 5. ONBOARD SYSTEM DIAGNOSTICS                                             */
/* ========================================================================= */
/** Active-LOW User Heartbeat LED (PC13 on BlackPill) */
#define MOBILITY_HEARTBEAT_PORT         GPIOC
#define MOBILITY_HEARTBEAT_PIN          GPIO_PIN_13
#define MOBILITY_HEARTBEAT_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#ifdef __cplusplus
}
#endif

#endif /* MOBILITY_PIN_CONFIG_H */
