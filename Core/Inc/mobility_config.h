#pragma once
#include "main.h"

/* Operational & Safety Parameters */
#define MOBILITY_ROS_DOMAIN_ID 42U
#define MOBILITY_DRIVE_CONFIGURED 0
#define MOBILITY_TRACK_WIDTH_M 0.50f /* Replace with measured left/right track separation. */
#define MOBILITY_FULL_SPEED_MPS 1.00f /* Replace with measured full-PWM ground speed. */
#define MOBILITY_MAX_LINEAR_MPS 0.40f
#define MOBILITY_MAX_ANGULAR_RPS 0.80f
#define MOBILITY_LEFT_INVERT 0
#define MOBILITY_RIGHT_INVERT 0
#define MOBILITY_COMMAND_TIMEOUT_MS 300U
#define MOBILITY_PWM_PERIOD 4799U /* TIM4 = 96 MHz / 4800 = 20 kHz */

/* Drivetrain (2x Cytron MDD10A Dual-Channel Drivers) */
/* Front-Left: TIM4 CH1 (PB6), DIR = PB4 */
#define MOTOR_FL_PWM_CHANNEL TIM_CHANNEL_1
#define MOTOR_FL_DIR_PORT    GPIOB
#define MOTOR_FL_DIR_PIN     GPIO_PIN_4

/* Rear-Left: TIM4 CH2 (PB7), DIR = PB5 */
#define MOTOR_RL_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR_RL_DIR_PORT    GPIOB
#define MOTOR_RL_DIR_PIN     GPIO_PIN_5

/* Front-Right: TIM4 CH3 (PB8), DIR = PC13 */
#define MOTOR_FR_PWM_CHANNEL TIM_CHANNEL_3
#define MOTOR_FR_DIR_PORT    GPIOC
#define MOTOR_FR_DIR_PIN     GPIO_PIN_13

/* Rear-Right: TIM4 CH4 (PB9), DIR = PC14 */
#define MOTOR_RR_PWM_CHANNEL TIM_CHANNEL_4
#define MOTOR_RR_DIR_PORT    GPIOC
#define MOTOR_RR_DIR_PIN     GPIO_PIN_14

/* Diagnostics & Indicators */
#define MOBILITY_BUZZER_PORT  GPIOB
#define MOBILITY_BUZZER_PIN   GPIO_PIN_12  /* Active Buzzer: Watchdog & alarm */

#define MOBILITY_LED1_PORT    GPIOB
#define MOBILITY_LED1_PIN     GPIO_PIN_13  /* Red LED: Watchdog trip status */

#define MOBILITY_LED2_PORT    GPIOB
#define MOBILITY_LED2_PIN     GPIO_PIN_14  /* Yellow LED: Motion active */

#define MOBILITY_LED3_PORT    GPIOB
#define MOBILITY_LED3_PIN     GPIO_PIN_15  /* Green LED: micro-ROS link ready */

/* MPU-6050 IMU on I2C2 (PB10 SCL, PB3 SDA) */
#define MPU6050_ADDRESS       (0x68U << 1)
