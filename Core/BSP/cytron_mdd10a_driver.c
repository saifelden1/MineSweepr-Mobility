/**
 * @file cytron_mdd10a_driver.c
 * @brief Concrete BSP Driver implementation for Cytron MDD10A.
 */

#include "cytron_mdd10a_driver.h"
#include <math.h>

extern TIM_HandleTypeDef htim4;

static bool s_initialized = false;

static float clamp_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

bool Cytron_MDD10A_Init(void)
{
    /* Configure DIR GPIO pins: PB12 (Left), PB13 (Right) */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MDD10A_LEFT_DIR_PIN | MDD10A_RIGHT_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Set default direction LOW and PWM to 0 */
    HAL_GPIO_WritePin(MDD10A_LEFT_DIR_PORT, MDD10A_LEFT_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MDD10A_RIGHT_DIR_PORT, MDD10A_RIGHT_DIR_PIN, GPIO_PIN_RESET);

    /* Start PWM generation on TIM4 CH1 & CH2 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    s_initialized = true;
    return true;
}

bool Cytron_MDD10A_SetSpeedNorm(motor_id_t motor, float speed_norm)
{
    if (motor >= MOTOR_ID_COUNT)
    {
        return false;
    }

    float clamped_speed = clamp_float(speed_norm, -1.0f, 1.0f);
    motor_dir_t dir = MOTOR_DIR_BRAKE;
    uint16_t duty_ticks = 0;

    if (clamped_speed > 0.001f)
    {
        dir = MOTOR_DIR_FORWARD;
        duty_ticks = (uint16_t)(clamped_speed * (float)MDD10A_PWM_MAX_TICKS + 0.5f);
    }
    else if (clamped_speed < -0.001f)
    {
        dir = MOTOR_DIR_REVERSE;
        duty_ticks = (uint16_t)((-clamped_speed) * (float)MDD10A_PWM_MAX_TICKS + 0.5f);
    }
    else
    {
        dir = MOTOR_DIR_BRAKE;
        duty_ticks = 0;
    }

    return Cytron_MDD10A_SetPwmDuty(motor, dir, duty_ticks);
}

bool Cytron_MDD10A_SetPwmDuty(motor_id_t motor, motor_dir_t dir, uint16_t duty_ticks)
{
    if (motor >= MOTOR_ID_COUNT)
    {
        return false;
    }

    if (duty_ticks > MDD10A_PWM_MAX_TICKS)
    {
        duty_ticks = MDD10A_PWM_MAX_TICKS;
    }

    GPIO_TypeDef *dir_port = (motor == MOTOR_ID_LEFT) ? MDD10A_LEFT_DIR_PORT : MDD10A_RIGHT_DIR_PORT;
    uint16_t dir_pin = (motor == MOTOR_ID_LEFT) ? MDD10A_LEFT_DIR_PIN : MDD10A_RIGHT_DIR_PIN;
    uint32_t tim_channel = (motor == MOTOR_ID_LEFT) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;

    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim4, tim_channel, duty_ticks);
            break;

        case MOTOR_DIR_REVERSE:
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim4, tim_channel, duty_ticks);
            break;

        case MOTOR_DIR_BRAKE:
        default:
            __HAL_TIM_SET_COMPARE(&htim4, tim_channel, 0);
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
            break;
    }

    return true;
}

bool Cytron_MDD10A_EmergencyStopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(MDD10A_LEFT_DIR_PORT, MDD10A_LEFT_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MDD10A_RIGHT_DIR_PORT, MDD10A_RIGHT_DIR_PIN, GPIO_PIN_RESET);

    return true;
}

static const motor_interface_t s_cytron_interface = {
    .init              = Cytron_MDD10A_Init,
    .set_speed_norm    = Cytron_MDD10A_SetSpeedNorm,
    .set_pwm_duty      = Cytron_MDD10A_SetPwmDuty,
    .emergency_stop_all= Cytron_MDD10A_EmergencyStopAll,
};

const motor_interface_t* Cytron_MDD10A_GetInterface(void)
{
    return &s_cytron_interface;
}
