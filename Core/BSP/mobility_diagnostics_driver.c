/**
 * @file mobility_diagnostics_driver.c
 * @brief Board Support Package Driver Implementation for Mobility Diagnostics.
 */

#include "mobility_diagnostics_driver.h"

static volatile bool s_watchdog_tripped = false;
static volatile bool s_beep_active = false;
static volatile uint32_t s_beep_end_tick = 0;
static bool s_diagnostics_initialized = false;

void Mobility_Diagnostics_Init(void)
{
    /* Enable GPIOB peripheral clock */
    MOBILITY_BUZZER_CLK_ENABLE();
    MOBILITY_LED1_CLK_ENABLE();
    MOBILITY_LED2_CLK_ENABLE();
    MOBILITY_LED3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Ensure initial output state is LOW (OFF) */
    HAL_GPIO_WritePin(MOBILITY_BUZZER_PORT, MOBILITY_BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOBILITY_LED1_PORT, MOBILITY_LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOBILITY_LED2_PORT, MOBILITY_LED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOBILITY_LED3_PORT, MOBILITY_LED3_PIN, GPIO_PIN_RESET);

    /* Initialize PB12 (Buzzer), PB13 (LED1), PB14 (LED2), PB15 (LED3) as push-pull outputs */
    GPIO_InitStruct.Pin = MOBILITY_BUZZER_PIN | MOBILITY_LED1_PIN | MOBILITY_LED2_PIN | MOBILITY_LED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    s_watchdog_tripped = false;
    s_beep_active = false;
    s_beep_end_tick = 0;
    s_diagnostics_initialized = true;
}

void Mobility_Diagnostics_SetLed(uint8_t led_id, bool state)
{
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (led_id)
    {
        case MOBILITY_LED_1:
            HAL_GPIO_WritePin(MOBILITY_LED1_PORT, MOBILITY_LED1_PIN, pin_state);
            break;

        case MOBILITY_LED_2:
            HAL_GPIO_WritePin(MOBILITY_LED2_PORT, MOBILITY_LED2_PIN, pin_state);
            break;

        case MOBILITY_LED_3:
            HAL_GPIO_WritePin(MOBILITY_LED3_PORT, MOBILITY_LED3_PIN, pin_state);
            break;

        default:
            break;
    }
}

void Mobility_Diagnostics_SetBuzzer(bool state)
{
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(MOBILITY_BUZZER_PORT, MOBILITY_BUZZER_PIN, pin_state);
}

void Mobility_Diagnostics_BuzzerBeep(uint32_t ms)
{
    if (ms == 0)
    {
        s_beep_active = false;
        if (!s_watchdog_tripped)
        {
            Mobility_Diagnostics_SetBuzzer(false);
        }
        return;
    }

    Mobility_Diagnostics_SetBuzzer(true);
    s_beep_end_tick = HAL_GetTick() + ms;
    s_beep_active = true;
}

void Mobility_Diagnostics_SetWatchdogAlert(bool tripped)
{
    s_watchdog_tripped = tripped;
    Mobility_Diagnostics_SetLed(MOBILITY_LED_1, tripped);

    if (tripped)
    {
        Mobility_Diagnostics_SetBuzzer(true);
    }
    else
    {
        if (!s_beep_active)
        {
            Mobility_Diagnostics_SetBuzzer(false);
        }
    }
}

void Mobility_Diagnostics_SetMotionActive(bool moving)
{
    Mobility_Diagnostics_SetLed(MOBILITY_LED_2, moving);
}

void Mobility_Diagnostics_SetRosConnected(bool connected)
{
    Mobility_Diagnostics_SetLed(MOBILITY_LED_3, connected);
}

void Mobility_Diagnostics_Update(void)
{
    if (s_beep_active)
    {
        if ((int32_t)(HAL_GetTick() - s_beep_end_tick) >= 0)
        {
            s_beep_active = false;
            if (!s_watchdog_tripped)
            {
                Mobility_Diagnostics_SetBuzzer(false);
            }
        }
    }
}
