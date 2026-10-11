/**
 * @file task_safety_watchdog.h
 * @brief Safety Watchdog Task for STM32_Mobility.
 */

#ifndef TASK_SAFETY_WATCHDOG_H
#define TASK_SAFETY_WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_SafetyWatchdog_Init(void);
void Task_SafetyWatchdog_Feed(void);
bool Task_SafetyWatchdog_IsTripped(void);
void Task_SafetyWatchdog_TriggerMine(void);
void Task_SafetyWatchdog_ClearMine(void);
void StartSafetyWatchdogTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SAFETY_WATCHDOG_H */
