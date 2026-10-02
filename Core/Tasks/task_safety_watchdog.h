/**
 * @file task_safety_watchdog.h
 * @brief FreeRTOS Safety Watchdog Task (< 200 ms Command Timeout & Mine Interlock).
 *
 * Implements highest-priority real-time task (osPriorityRealtime5, 10 ms period)
 * that zeros all 4 motor PWM outputs and clamps DIR pins within < 200 ms
 * if /cmd_vel ceases or mine trigger is detected.
 */

#ifndef TASK_SAFETY_WATCHDOG_H
#define TASK_SAFETY_WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WATCHDOG_TIMEOUT_MS_LIMIT   (200U)  /**< 200 ms command timeout */
#define WATCHDOG_TASK_PERIOD_MS     (10U)   /**< 10 ms monitoring loop (100 Hz) */

/**
 * @brief Initialize Safety Watchdog task internal state.
 */
void Task_SafetyWatchdog_Init(void);

/**
 * @brief Feed watchdog on receipt of valid /cmd_vel message.
 */
void Task_SafetyWatchdog_Feed(void);

/**
 * @brief Signal emergency mine detection event (zero latency trip).
 */
void Task_SafetyWatchdog_TriggerMine(void);

/**
 * @brief Clear emergency mine condition.
 */
void Task_SafetyWatchdog_ClearMine(void);

/**
 * @brief Query if watchdog is currently tripped into safe state.
 * @return true if tripped, false if healthy.
 */
bool Task_SafetyWatchdog_IsTripped(void);

/**
 * @brief Get human-readable reason for watchdog status ("HEALTHY", "TIMEOUT_...MS", "MINE_TRIGGER").
 */
const char* Task_SafetyWatchdog_GetReason(void);

/**
 * @brief FreeRTOS task entry point.
 */
void StartSafetyWatchdogTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SAFETY_WATCHDOG_H */
