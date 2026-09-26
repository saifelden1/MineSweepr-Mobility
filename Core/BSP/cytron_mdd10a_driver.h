/**
 * @file cytron_mdd10a_driver.h
 * @brief Concrete BSP Driver for Cytron MDD10A Dual Motor Driver.
 *
 * Implements motor_interface_t using TIM4 PWM (PB6/PB7 @ 20 kHz, ARR=4799)
 * and GPIO Direction pins (PB12/PB13).
 */

#ifndef CYTRON_MDD10A_DRIVER_H
#define CYTRON_MDD10A_DRIVER_H

#include "interfaces/motor_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MDD10A_PWM_MAX_TICKS       (4799U)   /**< ARR for 20 kHz @ 96 MHz */
#define MDD10A_LEFT_DIR_PORT       GPIOB
#define MDD10A_LEFT_DIR_PIN        GPIO_PIN_12
#define MDD10A_RIGHT_DIR_PORT      GPIOB
#define MDD10A_RIGHT_DIR_PIN       GPIO_PIN_13

/**
 * @brief Initialize Cytron MDD10A hardware (TIM4 PWM and DIR GPIOs).
 * @return true on success, false otherwise.
 */
bool Cytron_MDD10A_Init(void);

/**
 * @brief Set normalized motor speed [-1.0f .. +1.0f].
 * @param motor Motor identifier (MOTOR_ID_LEFT or MOTOR_ID_RIGHT).
 * @param speed_norm Speed normalized to [-1.0f .. 1.0f].
 * @return true on success, false on invalid parameter.
 */
bool Cytron_MDD10A_SetSpeedNorm(motor_id_t motor, float speed_norm);

/**
 * @brief Set raw PWM duty and direction ticks.
 * @param motor Motor identifier.
 * @param dir Direction (REVERSE, BRAKE, FORWARD).
 * @param duty_ticks Duty cycle in timer ticks [0 .. 4799].
 * @return true on success, false on invalid parameter.
 */
bool Cytron_MDD10A_SetPwmDuty(motor_id_t motor, motor_dir_t dir, uint16_t duty_ticks);

/**
 * @brief Emergency stop clamping all motor PWM outputs to 0 immediately.
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
