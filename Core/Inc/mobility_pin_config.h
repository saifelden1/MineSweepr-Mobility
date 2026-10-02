/**
 * @file mobility_pin_config.h
 * @brief Centralized Hardware Pin & Peripheral Configuration for STM32_Mobility ECU.
 *
 * Target: STM32F411CEU6 BlackPill (ARM Cortex-M4 @ 96 MHz)
 * System Clock: 96 MHz (25 MHz HSE PLL / 16 MHz HSI PLL)
 *
 * All GPIO pins, timer channels, bus peripherals, baud rates, and kinematic
 * physical constants are centralized in this header using clean #define macros.
 * Remapping any pin or peripheral requires modifying ONLY this header.
 */

#ifndef MOBILITY_PIN_CONFIG_H
#define MOBILITY_PIN_CONFIG_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. PINOUT SCHEME SELECTION                                                 */
/* ========================================================================== */
/**
 * Pinout Schemes:
 *  - SCHEME 1 (DEFAULT / RECOMMENDED):
 *      4 independent PWM channels on TIM4 CH1-4 (PB6, PB7, PB8, PB9 @ 20 kHz).
 *      MPU-6050 on I2C2 (PB10 SCL, PB3 SDA @ 400 kHz) to eliminate the
 *      hardware pin collision on PB6-PB9.
 *  - SCHEME 2 (SPLIT TIMERS):
 *      TIM4 CH1/2 (PB6/PB7) for Left motors, TIM3 CH3/4 (PB0/PB1) for Right.
 *      MPU-6050 on I2C1 (PB8 SCL, PB9 SDA @ 400 kHz).
 *  - SCHEME 3 (PAIRED PWM):
 *      TIM4 CH1 (PB6) for Left pair, TIM4 CH2 (PB7) for Right pair.
 *      MPU-6050 on I2C1 (PB8 SCL, PB9 SDA @ 400 kHz).
 */
#ifndef MOBILITY_PINOUT_SCHEME
#define MOBILITY_PINOUT_SCHEME 1
#endif

/* ========================================================================== */
/* 2. DRIVETRAIN PWM OUTPUTS (Cytron MDD10A 4-Motor Skit-Steer)               */
/* ========================================================================== */
#define MOTOR_PWM_MAX_TICKS         (4799U)  /**< ARR for 20 kHz PWM @ 96 MHz (PSC=0) */
#define MOTOR_PWM_FREQUENCY_HZ      (20000U) /**< 20 kHz ultrasonic carrier */

/* Front-Left Motor (FL) - Driver 1 Channel 1 */
#define MOTOR_FL_PWM_TIMER          TIM4
#define MOTOR_FL_PWM_CHANNEL        TIM_CHANNEL_1
#define MOTOR_FL_PWM_PORT           GPIOB
#define MOTOR_FL_PWM_PIN            GPIO_PIN_6          /**< Pin 14: TIM4_CH1 (AF2) */
#define MOTOR_FL_PWM_AF             GPIO_AF2_TIM4

/* Rear-Left Motor (RL) - Driver 1 Channel 2 */
#define MOTOR_RL_PWM_TIMER          TIM4
#define MOTOR_RL_PWM_CHANNEL        TIM_CHANNEL_2
#define MOTOR_RL_PWM_PORT           GPIOB
#define MOTOR_RL_PWM_PIN            GPIO_PIN_7          /**< Pin 15: TIM4_CH2 (AF2) */
#define MOTOR_RL_PWM_AF             GPIO_AF2_TIM4

#if (MOBILITY_PINOUT_SCHEME == 1)
/* Front-Right Motor (FR) - Driver 2 Channel 1 */
#define MOTOR_FR_PWM_TIMER          TIM4
#define MOTOR_FR_PWM_CHANNEL        TIM_CHANNEL_3
#define MOTOR_FR_PWM_PORT           GPIOB
#define MOTOR_FR_PWM_PIN            GPIO_PIN_8          /**< Pin 16: TIM4_CH3 (AF2) */
#define MOTOR_FR_PWM_AF             GPIO_AF2_TIM4

/* Rear-Right Motor (RR) - Driver 2 Channel 2 */
#define MOTOR_RR_PWM_TIMER          TIM4
#define MOTOR_RR_PWM_CHANNEL        TIM_CHANNEL_4
#define MOTOR_RR_PWM_PORT           GPIOB
#define MOTOR_RR_PWM_PIN            GPIO_PIN_9          /**< Pin 17: TIM4_CH4 (AF2) */
#define MOTOR_RR_PWM_AF             GPIO_AF2_TIM4

#elif (MOBILITY_PINOUT_SCHEME == 2)
/* Split Timers: Right side on TIM3 CH3/CH4 (PB0/PB1) */
#define MOTOR_FR_PWM_TIMER          TIM3
#define MOTOR_FR_PWM_CHANNEL        TIM_CHANNEL_3
#define MOTOR_FR_PWM_PORT           GPIOB
#define MOTOR_FR_PWM_PIN            GPIO_PIN_0          /**< Pin 18: TIM3_CH3 (AF2) */
#define MOTOR_FR_PWM_AF             GPIO_AF2_TIM3

#define MOTOR_RR_PWM_TIMER          TIM3
#define MOTOR_RR_PWM_CHANNEL        TIM_CHANNEL_4
#define MOTOR_RR_PWM_PORT           GPIOB
#define MOTOR_RR_PWM_PIN            GPIO_PIN_1          /**< Pin 19: TIM3_CH4 (AF2) */
#define MOTOR_RR_PWM_AF             GPIO_AF2_TIM3

#elif (MOBILITY_PINOUT_SCHEME == 3)
/* Paired PWM: Right side shared on TIM4 CH2 */
#define MOTOR_FR_PWM_TIMER          TIM4
#define MOTOR_FR_PWM_CHANNEL        TIM_CHANNEL_2
#define MOTOR_FR_PWM_PORT           GPIOB
#define MOTOR_FR_PWM_PIN            GPIO_PIN_7
#define MOTOR_FR_PWM_AF             GPIO_AF2_TIM4

#define MOTOR_RR_PWM_TIMER          TIM4
#define MOTOR_RR_PWM_CHANNEL        TIM_CHANNEL_2
#define MOTOR_RR_PWM_PORT           GPIOB
#define MOTOR_RR_PWM_PIN            GPIO_PIN_7
#define MOTOR_RR_PWM_AF             GPIO_AF2_TIM4
#endif

/* ========================================================================== */
/* 3. DRIVETRAIN DIRECTION GPIOs (Cytron MDD10A DIR Lines)                    */
/* ========================================================================== */
/* Front-Left Motor DIR (Driver 1 DIR1) */
#define MOTOR_FL_DIR_PORT           GPIOB
#define MOTOR_FL_DIR_PIN            GPIO_PIN_4          /**< Pin 12: PB4 */

/* Rear-Left Motor DIR (Driver 1 DIR2) */
#define MOTOR_RL_DIR_PORT           GPIOB
#define MOTOR_RL_DIR_PIN            GPIO_PIN_5          /**< Pin 13: PB5 */

/* Front-Right Motor DIR (Driver 2 DIR1) */
#define MOTOR_FR_DIR_PORT           GPIOC
#define MOTOR_FR_DIR_PIN            GPIO_PIN_13         /**< Pin 22: PC13 */

/* Rear-Right Motor DIR (Driver 2 DIR2) */
#define MOTOR_RR_DIR_PORT           GPIOC
#define MOTOR_RR_DIR_PIN            GPIO_PIN_14         /**< Pin 23: PC14 */

/* Direction Inversion Polarity (0 = normal, 1 = inverted) */
#define MOTOR_FL_INVERT_DIR         (0)
#define MOTOR_RL_INVERT_DIR         (0)
#define MOTOR_FR_INVERT_DIR         (1)                 /**< Right side motors inverted physically */
#define MOTOR_RR_INVERT_DIR         (1)

/* Backwards-compatibility aliases */
#define MDD10A_LEFT_DIR_PORT        MOTOR_FL_DIR_PORT
#define MDD10A_LEFT_DIR_PIN         MOTOR_FL_DIR_PIN
#define MDD10A_RIGHT_DIR_PORT       MOTOR_FR_DIR_PORT
#define MDD10A_RIGHT_DIR_PIN        MOTOR_FR_DIR_PIN

/* ========================================================================== */
/* 4. MPU-6050 6-DOF IMU CONFIGURATION                                        */
/* ========================================================================== */
#if (MOBILITY_PINOUT_SCHEME == 1)
/* Scheme 1: I2C2 on PB10 (SCL) and PB3 (SDA) - Zero conflict with TIM4 */
#define IMU_I2C_PORT                I2C2
#define IMU_I2C_INSTANCE            I2C2
#define IMU_I2C_HANDLE              hi2c2
#define IMU_I2C_SCL_PORT            GPIOB
#define IMU_I2C_SCL_PIN             GPIO_PIN_10         /**< Pin 36: PB10 (AF4) */
#define IMU_I2C_SCL_AF              GPIO_AF4_I2C2
#define IMU_I2C_SDA_PORT            GPIOB
#define IMU_I2C_SDA_PIN             GPIO_PIN_3          /**< Pin 11: PB3 (AF9) */
#define IMU_I2C_SDA_AF              GPIO_AF9_I2C2
#define IMU_I2C_CLK_ENABLE()        __HAL_RCC_I2C2_CLK_ENABLE()
#define IMU_I2C_CLK_DISABLE()       __HAL_RCC_I2C2_CLK_DISABLE()
#else
/* Fallback Scheme 2 / 3: I2C1 on PB8 (SCL) and PB9 (SDA) */
#define IMU_I2C_PORT                I2C1
#define IMU_I2C_INSTANCE            I2C1
#define IMU_I2C_HANDLE              hi2c1
#define IMU_I2C_SCL_PORT            GPIOB
#define IMU_I2C_SCL_PIN             GPIO_PIN_8          /**< Pin 16: PB8 (AF4) */
#define IMU_I2C_SCL_AF              GPIO_AF4_I2C1
#define IMU_I2C_SDA_PORT            GPIOB
#define IMU_I2C_SDA_PIN             GPIO_PIN_9          /**< Pin 17: PB9 (AF4) */
#define IMU_I2C_SDA_AF              GPIO_AF4_I2C1
#define IMU_I2C_CLK_ENABLE()        __HAL_RCC_I2C1_CLK_ENABLE()
#define IMU_I2C_CLK_DISABLE()       __HAL_RCC_I2C1_CLK_DISABLE()
#endif

#define IMU_I2C_SPEED_HZ            (400000U)           /**< 400 kHz Fast Mode */

/* ========================================================================== */
/* 5. u-blox NEO-6M GPS RECEIVER CONFIGURATION                                */
/* ========================================================================== */
#define GPS_UART_PORT               USART2
#define GPS_UART_INSTANCE           USART2
#define GPS_UART_HANDLE             huart2
#define GPS_UART_BAUDRATE           (9600U)
#define GPS_UART_TX_PORT            GPIOA
#define GPS_UART_TX_PIN             GPIO_PIN_2          /**< Pin 27: PA2 (AF7) */
#define GPS_UART_RX_PORT            GPIOA
#define GPS_UART_RX_PIN             GPIO_PIN_3          /**< Pin 28: PA3 (AF7) */
#define GPS_UART_AF                 GPIO_AF7_USART2
#define GPS_DMA_RX_STREAM           DMA1_Stream5
#define GPS_DMA_RX_CHANNEL          DMA_CHANNEL_4
#define GPS_DMA_BUF_SIZE            (512U)
#define GPS_NMEA_MAX_SENTENCE_LEN   (96U)

/* ========================================================================== */
/* 6. micro-ROS SERIAL TRANSPORT (UART DMA)                                  */
/* ========================================================================== */
#define MICROROS_UART_PORT          USART1
#define MICROROS_UART_INSTANCE      USART1
#define MICROROS_UART_HANDLE        huart1
#define MICROROS_UART_BAUDRATE      (921600U)           /**< High-speed serial link */
#define MICROROS_UART_TX_PORT       GPIOA
#define MICROROS_UART_TX_PIN        GPIO_PIN_9          /**< Pin 6: PA9 (AF7) */
#define MICROROS_UART_RX_PORT       GPIOA
#define MICROROS_UART_RX_PIN        GPIO_PIN_10         /**< Pin 7: PA10 (AF7) */
#define MICROROS_UART_AF            GPIO_AF7_USART1
#define MICROROS_DMA_RX_STREAM      DMA2_Stream2
#define MICROROS_DMA_RX_CHANNEL     DMA_CHANNEL_4
#define MICROROS_DMA_BUF_SIZE       (2048U)

/* ========================================================================== */
/* 7. ROVER PHYSICAL GEOMETRY & KINEMATICS CONSTANTS                         */
/* ========================================================================== */
#define ROBOT_TRACK_GAUGE_M         (0.55f)             /**< Track width L = 0.55 m */
#define ROBOT_WHEEL_RADIUS_M        (0.13f)             /**< Wheel radius R = 0.13 m */
#define ROBOT_MAX_LINEAR_VEL_M_S    (1.00f)             /**< Max forward speed = 1.0 m/s */
#define ROBOT_MAX_ANGULAR_VEL_RAD_S (3.00f)             /**< Max yaw rate = 3.0 rad/s */
#define MOTOR_MIN_DEADBAND_DUTY     (0.08f)             /**< 8% minimum duty cycle */

#ifdef __cplusplus
}
#endif

#endif /* MOBILITY_PIN_CONFIG_H */
