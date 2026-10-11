/**
 * @file cytron_mdd10a_driver.c
 * @brief 4-Motor Dual Cytron MDD10A Motor Driver Implementation.
 */

#include "cytron_mdd10a_driver.h"
#include <math.h>

static TIM_HandleTypeDef *s_htim = NULL;

static void set_single_motor(uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, float duty, bool is_right)
{
    if (s_htim == NULL) return;

    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint32_t ccr = 0;
    GPIO_PinState pin_state = GPIO_PIN_RESET;

    if (fabsf(duty) <= 0.001f)
    {
        ccr = 0;
        pin_state = GPIO_PIN_RESET;
    }
    else if (duty > 0.0f)
    {
        ccr = (uint32_t)(fabsf(duty) * (float)MOTOR_PWM_MAX_TICKS);
        if (ccr > MOTOR_PWM_MAX_TICKS) ccr = MOTOR_PWM_MAX_TICKS;

        /* Left motor: HIGH = Forward; Right motor: LOW = Forward (hardware polarity) */
        pin_state = is_right ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else
    {
        ccr = (uint32_t)(fabsf(duty) * (float)MOTOR_PWM_MAX_TICKS);
        if (ccr > MOTOR_PWM_MAX_TICKS) ccr = MOTOR_PWM_MAX_TICKS;

        /* Left motor: LOW = Reverse; Right motor: HIGH = Reverse */
        pin_state = is_right ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(dir_port, dir_pin, pin_state);
    __HAL_TIM_SET_COMPARE(s_htim, channel, ccr);
}

void Cytron_MDD10A_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;

    /* Start PWM on all 4 channels */
    if (s_htim != NULL)
    {
        HAL_TIM_PWM_Start(s_htim, MOTOR_FL_PWM_CHANNEL);
        HAL_TIM_PWM_Start(s_htim, MOTOR_RL_PWM_CHANNEL);
        HAL_TIM_PWM_Start(s_htim, MOTOR_FR_PWM_CHANNEL);
        HAL_TIM_PWM_Start(s_htim, MOTOR_RR_PWM_CHANNEL);
    }

    Cytron_MDD10A_BrakeAll();
}

void Cytron_MDD10A_SetMotors(float fl_duty, float rl_duty, float fr_duty, float rr_duty)
{
    /* Left Pair (FL, RL) */
    set_single_motor(MOTOR_FL_PWM_CHANNEL, MOTOR_FL_DIR_PORT, MOTOR_FL_DIR_PIN, fl_duty, false);
    set_single_motor(MOTOR_RL_PWM_CHANNEL, MOTOR_RL_DIR_PORT, MOTOR_RL_DIR_PIN, rl_duty, false);

    /* Right Pair (FR, RR) */
    set_single_motor(MOTOR_FR_PWM_CHANNEL, MOTOR_FR_DIR_PORT, MOTOR_FR_DIR_PIN, fr_duty, true);
    set_single_motor(MOTOR_RR_PWM_CHANNEL, MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN, rr_duty, true);
}

void Cytron_MDD10A_BrakeAll(void)
{
    if (s_htim != NULL)
    {
        __HAL_TIM_SET_COMPARE(s_htim, MOTOR_FL_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(s_htim, MOTOR_RL_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(s_htim, MOTOR_FR_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(s_htim, MOTOR_RR_PWM_CHANNEL, 0);
    }

    HAL_GPIO_WritePin(MOTOR_FL_DIR_PORT, MOTOR_FL_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RL_DIR_PORT, MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_FR_DIR_PORT, MOTOR_FR_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);
}
