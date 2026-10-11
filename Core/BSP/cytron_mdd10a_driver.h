/**
 * @file cytron_mdd10a_driver.h
 * @brief 4-Motor Dual Cytron MDD10A Motor Driver for STM32_Mobility.
 */

#ifndef CYTRON_MDD10A_DRIVER_H
#define CYTRON_MDD10A_DRIVER_H

#include "mobility_pin_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_DIR_REVERSE = -1,
    MOTOR_DIR_BRAKE   = 0,
    MOTOR_DIR_FORWARD = 1
} motor_direction_t;

/**
 * @brief Initialize TIM4 PWM channels and DIR GPIO pins.
 * @param htim Pointer to TIM4 handle.
 */
void Cytron_MDD10A_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Set individual duty cycle for each of the 4 motors.
 * @param fl_duty Front-Left duty [-1.0f .. 1.0f]
 * @param rl_duty Rear-Left duty  [-1.0f .. 1.0f]
 * @param fr_duty Front-Right duty [-1.0f .. 1.0f]
 * @param rr_duty Rear-Right duty  [-1.0f .. 1.0f]
 */
void Cytron_MDD10A_SetMotors(float fl_duty, float rl_duty, float fr_duty, float rr_duty);

/**
 * @brief Immediately brake all 4 motors (zero PWM duty).
 */
void Cytron_MDD10A_BrakeAll(void);

#ifdef __cplusplus
}
#endif

#endif /* CYTRON_MDD10A_DRIVER_H */
