/**
 * @file cytron_mdd10a_driver.h
 * @brief Concrete BSP Driver for Cytron MDD10A Dual Motor Drivers (4-Motor Drivetrain).
 *
 * Implements motor_interface_t driving 4 independent motors (FL, RL, FR, RR)
 * via TIM4 PWM channels 1-4 (@ 20 kHz, ARR=4799) and 4 GPIO Direction pins
 * (PB4, PB5, PC13, PC14) as configured in mobility_pin_config.h.
 */

#ifndef CYTRON_MDD10A_DRIVER_H
#define CYTRON_MDD10A_DRIVER_H

#include "interfaces/motor_interface.h"
#include "mobility_pin_config.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MDD10A_PWM_MAX_TICKS
#define MDD10A_PWM_MAX_TICKS       MOTOR_PWM_MAX_TICKS
#endif

/**
 * @brief Initialize Cytron MDD10A hardware (4 PWM channels on TIM4 and 4 DIR GPIOs).
 * @return true on success, false otherwise.
 */
bool Cytron_MDD10A_Init(void);

/**
 * @brief Set normalized speed for an individual motor [-1.0f .. +1.0f].
 * @param motor Motor identifier (MOTOR_ID_FL, MOTOR_ID_RL, MOTOR_ID_FR, MOTOR_ID_RR).
 * @param speed_norm Speed normalized to [-1.0f .. 1.0f].
 * @return true on success, false on invalid parameter.
 */
bool Cytron_MDD10A_SetSpeedNorm(motor_id_t motor, float speed_norm);

/**
 * @brief Set differential drive speeds for left and right pairs simultaneously.
 *        FL & RL receive left_norm; FR & RR receive right_norm.
 * @param left_norm Normalized speed for left wheels [-1.0f .. 1.0f].
 * @param right_norm Normalized speed for right wheels [-1.0f .. 1.0f].
 * @return true on success.
 */
bool Cytron_MDD10A_SetDiffDriveSpeeds(float left_norm, float right_norm);

/**
 * @brief Set raw PWM duty and direction for an individual motor.
 * @param motor Motor identifier.
 * @param dir Direction (REVERSE, BRAKE, FORWARD).
 * @param duty_ticks Duty cycle in timer ticks [0 .. MDD10A_PWM_MAX_TICKS].
 * @return true on success, false on invalid parameter.
 */
bool Cytron_MDD10A_SetPwmDuty(motor_id_t motor, motor_dir_t dir, uint16_t duty_ticks);

/**
 * @brief Emergency stop clamping all 4 motor PWM outputs to 0 immediately
 *        and setting all DIR lines to safe RESET.
 * @return true on success.
 */
bool Cytron_MDD10A_EmergencyStopAll(void);

/**
 * @brief Get the motor_interface_t function pointer table for this driver.
 * @return Pointer to motor_interface_t instance.
 */
const motor_interface_t* Cytron_MDD10A_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* CYTRON_MDD10A_DRIVER_H */
