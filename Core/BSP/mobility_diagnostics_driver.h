/**
 * @file mobility_diagnostics_driver.h
 * @brief Board Support Package Driver for Mobility Diagnostics (LEDs & Buzzer).
 *
 * Microcontroller: STM32F411CEU6 BlackPill (ARM Cortex-M4F @ 96 MHz)
 * Hardware Assignments (R3):
 *   - PB12: Audible Buzzer (Active alarm / watchdog trip)
 *   - PB13: LED 1 (Red) - Watchdog trip / Error alert
 *   - PB14: LED 2 (Yellow) - Drivetrain motion active
 *   - PB15: LED 3 (Green) - System ready & micro-ROS connected
 */

#ifndef MOBILITY_DIAGNOSTICS_DRIVER_H
#define MOBILITY_DIAGNOSTICS_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "mobility_pin_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Logical identifiers for mobility status LEDs.
 */
typedef enum {
    MOBILITY_LED_1      = 1, /**< PB13 - Red: Watchdog trip / Error alert */
    MOBILITY_LED_2      = 2, /**< PB14 - Yellow: Drivetrain motion active */
    MOBILITY_LED_3      = 3, /**< PB15 - Green: System ready & micro-ROS connected */

    MOBILITY_LED_RED    = 1, /**< Alias for LED 1 */
    MOBILITY_LED_YELLOW = 2, /**< Alias for LED 2 */
    MOBILITY_LED_GREEN  = 3  /**< Alias for LED 3 */
} mobility_led_id_t;

/**
 * @brief Initialize PB12, PB13, PB14, PB15 as GPIO push-pull outputs.
 */
void Mobility_Diagnostics_Init(void);

/**
 * @brief Set output state of a specific status LED.
 * @param led_id 1 (Red), 2 (Yellow), 3 (Green).
 * @param state true = ON, false = OFF.
 */
void Mobility_Diagnostics_SetLed(uint8_t led_id, bool state);

/**
 * @brief Set output state of the audible buzzer.
 * @param state true = ON, false = OFF.
 */
void Mobility_Diagnostics_SetBuzzer(bool state);

/**
 * @brief Sound buzzer for a specified duration in milliseconds (non-blocking).
 * @param ms Duration in ms. If ms == 0, buzzer is silenced immediately.
 */
void Mobility_Diagnostics_BuzzerBeep(uint32_t ms);

/**
 * @brief Watchdog trip / Error alert status indicator.
 * Controls LED 1 (Red) and Buzzer.
 * @param tripped true if watchdog tripped or error active; false otherwise.
 */
void Mobility_Diagnostics_SetWatchdogAlert(bool tripped);

/**
 * @brief Drivetrain motion active status indicator.
 * Controls LED 2 (Yellow).
 * @param moving true if drivetrain motion is actively commanded.
 */
void Mobility_Diagnostics_SetMotionActive(bool moving);

/**
 * @brief micro-ROS connection & system ready status indicator.
 * Controls LED 3 (Green).
 * @param connected true if micro-ROS agent connected and system ready.
 */
void Mobility_Diagnostics_SetRosConnected(bool connected);

/**
 * @brief Periodic update function for non-blocking buzzer duration management.
 * Should be called periodically (e.g. from a task loop or timer tick).
 */
void Mobility_Diagnostics_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* MOBILITY_DIAGNOSTICS_DRIVER_H */
