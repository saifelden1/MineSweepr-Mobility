/**
 * @file task_safety_watchdog.c
 * @brief Safety Watchdog Task Implementation for STM32_Mobility.
 *
 * Runs at highest RTOS priority. Zeroes all motor outputs if /cmd_vel
 * has not arrived within 200 ms.
 */

#include "task_safety_watchdog.h"
#include "cytron_mdd10a_driver.h"
#include "mobility_pin_config.h"
#include "cmsis_os.h"

static volatile uint32_t s_last_feed_tick = 0;
static volatile bool s_is_tripped = false;

void Task_SafetyWatchdog_Init(void)
{
    s_last_feed_tick = osKernelGetTickCount();
    s_is_tripped = false;
}

void Task_SafetyWatchdog_Feed(void)
{
    s_last_feed_tick = osKernelGetTickCount();
    s_is_tripped = false;
}

bool Task_SafetyWatchdog_IsTripped(void)
{
    return s_is_tripped;
}

void StartSafetyWatchdogTask(void *argument)
{
    (void)argument;
    Task_SafetyWatchdog_Init();

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        uint32_t elapsed_ms = (now >= s_last_feed_tick) ? 
                              (now - s_last_feed_tick) : 
                              (0xFFFFFFFFU - s_last_feed_tick + now + 1);

        if (elapsed_ms > WATCHDOG_TIMEOUT_MS)
        {
            if (!s_is_tripped)
            {
                Cytron_MDD10A_BrakeAll();
                s_is_tripped = true;
            }
        }

        osDelay(10); // Check every 10 ms
    }
}
