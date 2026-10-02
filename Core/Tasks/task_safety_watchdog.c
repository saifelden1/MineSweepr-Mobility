/**
 * @file task_safety_watchdog.c
 * @brief FreeRTOS Safety Watchdog Task Implementation (< 200 ms Command Timeout).
 */

#include "task_safety_watchdog.h"
#include "cytron_mdd10a_driver.h"
#include "cmsis_os.h"
#include <stdio.h>

static volatile uint32_t s_last_feed_tick = 0;
static volatile bool s_mine_triggered = false;
static volatile bool s_is_tripped = false;
static char s_reason_str[32] = "INITIALIZING";

void Task_SafetyWatchdog_Init(void)
{
    s_last_feed_tick = xTaskGetTickCount();
    s_mine_triggered = false;
    s_is_tripped = false;
    snprintf(s_reason_str, sizeof(s_reason_str), "HEALTHY");
}

void Task_SafetyWatchdog_Feed(void)
{
    s_last_feed_tick = xTaskGetTickCount();
    if (!s_mine_triggered) {
        s_is_tripped = false;
        snprintf(s_reason_str, sizeof(s_reason_str), "HEALTHY");
    }
}

void Task_SafetyWatchdog_TriggerMine(void)
{
    s_mine_triggered = true;
    s_is_tripped = true;
    snprintf(s_reason_str, sizeof(s_reason_str), "MINE_TRIGGER");
    /* Immediately zero all 4 motor PWM outputs */
    Cytron_MDD10A_EmergencyStopAll();
}

void Task_SafetyWatchdog_ClearMine(void)
{
    s_mine_triggered = false;
    /* Do not immediately untrip; next Feed will restore healthy */
}

bool Task_SafetyWatchdog_IsTripped(void)
{
    return s_is_tripped;
}

const char* Task_SafetyWatchdog_GetReason(void)
{
    return s_reason_str;
}

void StartSafetyWatchdogTask(void *argument)
{
    (void)argument;
    Task_SafetyWatchdog_Init();

    for (;;) {
        uint32_t current_tick = xTaskGetTickCount();
        uint32_t elapsed_ms = (current_tick - s_last_feed_tick) * portTICK_PERIOD_MS;

        if (s_mine_triggered) {
            s_is_tripped = true;
            snprintf(s_reason_str, sizeof(s_reason_str), "MINE_TRIGGER");
            Cytron_MDD10A_EmergencyStopAll();
        } else if (elapsed_ms > WATCHDOG_TIMEOUT_MS_LIMIT) {
            s_is_tripped = true;
            snprintf(s_reason_str, sizeof(s_reason_str), "TIMEOUT_%luMS", (unsigned long)elapsed_ms);
            /* Command silence > 200 ms: zero all 4 motor PWM outputs */
            Cytron_MDD10A_EmergencyStopAll();
        } else {
            s_is_tripped = false;
            snprintf(s_reason_str, sizeof(s_reason_str), "HEALTHY");
        }

        osDelay(WATCHDOG_TASK_PERIOD_MS);
    }
}
