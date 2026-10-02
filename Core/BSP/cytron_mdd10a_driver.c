/**
 * @file cytron_mdd10a_driver.c
 * @brief Concrete BSP Driver implementation for Cytron MDD10A (4-Motor Drivetrain).
 */

#include "cytron_mdd10a_driver.h"
#include <math.h>

extern TIM_HandleTypeDef htim4;
#if (MOBILITY_PINOUT_SCHEME == 2)
extern TIM_HandleTypeDef htim3;
#endif

static bool s_initialized = false;

static float clamp_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

bool Cytron_MDD10A_Init(void)
{
    /* Enable GPIO Port clocks for direction control pins */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* Initialize FL DIR Pin */
    GPIO_InitStruct.Pin = MOTOR_FL_DIR_PIN;
    HAL_GPIO_Init(MOTOR_FL_DIR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MOTOR_FL_DIR_PORT, MOTOR_FL_DIR_PIN, GPIO_PIN_RESET);

    /* Initialize RL DIR Pin */
    GPIO_InitStruct.Pin = MOTOR_RL_DIR_PIN;
    HAL_GPIO_Init(MOTOR_RL_DIR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MOTOR_RL_DIR_PORT, MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);

    /* Initialize FR DIR Pin */
    GPIO_InitStruct.Pin = MOTOR_FR_DIR_PIN;
    HAL_GPIO_Init(MOTOR_FR_DIR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MOTOR_FR_DIR_PORT, MOTOR_FR_DIR_PIN, GPIO_PIN_RESET);

    /* Initialize RR DIR Pin */
    GPIO_InitStruct.Pin = MOTOR_RR_DIR_PIN;
    HAL_GPIO_Init(MOTOR_RR_DIR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);

    /* Start PWM generation on all 4 channels */
    HAL_TIM_PWM_Start(&htim4, MOTOR_FL_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, MOTOR_RL_PWM_CHANNEL);
#if (MOBILITY_PINOUT_SCHEME == 2)
    HAL_TIM_PWM_Start(&htim3, MOTOR_FR_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, MOTOR_RR_PWM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_FR_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_RR_PWM_CHANNEL, 0);
#else
    HAL_TIM_PWM_Start(&htim4, MOTOR_FR_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, MOTOR_RR_PWM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_FR_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RR_PWM_CHANNEL, 0);
#endif

    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_FL_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RL_PWM_CHANNEL, 0);

    s_initialized = true;
    return true;
}

bool Cytron_MDD10A_SetSpeedNorm(motor_id_t motor, float speed_norm)
{
    if (motor >= MOTOR_ID_COUNT)
    {
        return false;
    }

    /* Apply hardware directional mounting polarity inversion */
    float effective_speed = speed_norm;
    switch (motor)
    {
        case MOTOR_ID_FL:
            if (MOTOR_FL_INVERT_DIR) effective_speed = -effective_speed;
            break;
        case MOTOR_ID_RL:
            if (MOTOR_RL_INVERT_DIR) effective_speed = -effective_speed;
            break;
        case MOTOR_ID_FR:
            if (MOTOR_FR_INVERT_DIR) effective_speed = -effective_speed;
            break;
        case MOTOR_ID_RR:
            if (MOTOR_RR_INVERT_DIR) effective_speed = -effective_speed;
            break;
        default:
            break;
    }

    float clamped_speed = clamp_float(effective_speed, -1.0f, 1.0f);
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

bool Cytron_MDD10A_SetDiffDriveSpeeds(float left_norm, float right_norm)
{
    bool ok = true;
    ok &= Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_FL, left_norm);
    ok &= Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_RL, left_norm);
    ok &= Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_FR, right_norm);
    ok &= Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_RR, right_norm);
    return ok;
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

    GPIO_TypeDef *dir_port = NULL;
    uint16_t dir_pin = 0;
    TIM_HandleTypeDef *timer = &htim4;
    uint32_t tim_channel = 0;

    switch (motor)
    {
        case MOTOR_ID_FL:
            dir_port    = MOTOR_FL_DIR_PORT;
            dir_pin     = MOTOR_FL_DIR_PIN;
            timer       = &htim4;
            tim_channel = MOTOR_FL_PWM_CHANNEL;
            break;

        case MOTOR_ID_RL:
            dir_port    = MOTOR_RL_DIR_PORT;
            dir_pin     = MOTOR_RL_DIR_PIN;
            timer       = &htim4;
            tim_channel = MOTOR_RL_PWM_CHANNEL;
            break;

        case MOTOR_ID_FR:
            dir_port    = MOTOR_FR_DIR_PORT;
            dir_pin     = MOTOR_FR_DIR_PIN;
#if (MOBILITY_PINOUT_SCHEME == 2)
            timer       = &htim3;
#else
            timer       = &htim4;
#endif
            tim_channel = MOTOR_FR_PWM_CHANNEL;
            break;

        case MOTOR_ID_RR:
            dir_port    = MOTOR_RR_DIR_PORT;
            dir_pin     = MOTOR_RR_DIR_PIN;
#if (MOBILITY_PINOUT_SCHEME == 2)
            timer       = &htim3;
#else
            timer       = &htim4;
#endif
            tim_channel = MOTOR_RR_PWM_CHANNEL;
            break;

        default:
            return false;
    }

    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(timer, tim_channel, duty_ticks);
            break;

        case MOTOR_DIR_REVERSE:
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(timer, tim_channel, duty_ticks);
            break;

        case MOTOR_DIR_BRAKE:
        default:
            __HAL_TIM_SET_COMPARE(timer, tim_channel, 0);
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
            break;
    }

    return true;
}

bool Cytron_MDD10A_EmergencyStopAll(void)
{
    /* Immediately clamp all PWM channels to 0 duty cycle */
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_FL_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RL_PWM_CHANNEL, 0);
#if (MOBILITY_PINOUT_SCHEME == 2)
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_FR_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_RR_PWM_CHANNEL, 0);
#else
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_FR_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RR_PWM_CHANNEL, 0);
#endif

    /* Reset all direction pins to LOW */
    HAL_GPIO_WritePin(MOTOR_FL_DIR_PORT, MOTOR_FL_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RL_DIR_PORT, MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_FR_DIR_PORT, MOTOR_FR_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);

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
