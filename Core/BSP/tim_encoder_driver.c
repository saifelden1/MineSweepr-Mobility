/**
 * @file tim_encoder_driver.c
 * @brief Concrete BSP Driver implementation for Dual Quadrature Encoders.
 */

#include "tim_encoder_driver.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct {
    uint32_t last_raw_cnt;
    int64_t  total_ticks;
    int32_t  delta_ticks;
    float    speed_rpm;
    float    linear_velocity_m_s;
} encoder_channel_state_t;

static encoder_channel_state_t s_channels[ENCODER_ID_COUNT];
static bool s_initialized = false;

bool TIM_Encoder_Init(void)
{
    /* Start encoder mode on TIM2 (PA0/PA1, 32-bit) */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Start encoder mode on TIM3 (PA6/PA7, 16-bit) */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    for (int i = 0; i < (int)ENCODER_ID_COUNT; i++)
    {
        s_channels[i].last_raw_cnt = 0;
        s_channels[i].total_ticks = 0;
        s_channels[i].delta_ticks = 0;
        s_channels[i].speed_rpm = 0.0f;
        s_channels[i].linear_velocity_m_s = 0.0f;
    }

    s_initialized = true;
    return true;
}

void TIM_Encoder_Update(uint32_t delta_time_us)
{
    if (!s_initialized || delta_time_us == 0)
    {
        return;
    }

    float dt_sec = (float)delta_time_us / 1000000.0f;

    /* 1. Left Channel: TIM2 (32-bit hardware counter) */
    uint32_t raw_cnt_left = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta_left = (int32_t)(raw_cnt_left - s_channels[ENCODER_ID_LEFT].last_raw_cnt);
    s_channels[ENCODER_ID_LEFT].last_raw_cnt = raw_cnt_left;
    s_channels[ENCODER_ID_LEFT].delta_ticks = delta_left;
    s_channels[ENCODER_ID_LEFT].total_ticks += delta_left;

    float revs_left = (float)delta_left / ENCODER_TICKS_PER_REV_F;
    s_channels[ENCODER_ID_LEFT].speed_rpm = (revs_left / dt_sec) * 60.0f;
    s_channels[ENCODER_ID_LEFT].linear_velocity_m_s = ((float)delta_left * METERS_PER_TICK_F) / dt_sec;

    /* 2. Right Channel: TIM3 (16-bit counter with two's complement rollover) */
    uint16_t raw_cnt_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t prev_cnt_right = (uint16_t)s_channels[ENCODER_ID_RIGHT].last_raw_cnt;
    int16_t delta_right_16 = (int16_t)(raw_cnt_right - prev_cnt_right);
    int32_t delta_right = (int32_t)delta_right_16;

    s_channels[ENCODER_ID_RIGHT].last_raw_cnt = (uint32_t)raw_cnt_right;
    s_channels[ENCODER_ID_RIGHT].delta_ticks = delta_right;
    s_channels[ENCODER_ID_RIGHT].total_ticks += delta_right;

    float revs_right = (float)delta_right / ENCODER_TICKS_PER_REV_F;
    s_channels[ENCODER_ID_RIGHT].speed_rpm = (revs_right / dt_sec) * 60.0f;
    s_channels[ENCODER_ID_RIGHT].linear_velocity_m_s = ((float)delta_right * METERS_PER_TICK_F) / dt_sec;
}

int64_t TIM_Encoder_GetTotalTicks(encoder_id_t id)
{
    if (id >= ENCODER_ID_COUNT)
    {
        return 0;
    }
    return s_channels[id].total_ticks;
}

int32_t TIM_Encoder_GetDeltaTicks(encoder_id_t id)
{
    if (id >= ENCODER_ID_COUNT)
    {
        return 0;
    }
    return s_channels[id].delta_ticks;
}

float TIM_Encoder_GetLinearVelocity(encoder_id_t id)
{
    if (id >= ENCODER_ID_COUNT)
    {
        return 0.0f;
    }
    return s_channels[id].linear_velocity_m_s;
}

void TIM_Encoder_Reset(encoder_id_t id)
{
    if (id == ENCODER_ID_LEFT)
    {
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        s_channels[ENCODER_ID_LEFT].last_raw_cnt = 0;
        s_channels[ENCODER_ID_LEFT].total_ticks = 0;
        s_channels[ENCODER_ID_LEFT].delta_ticks = 0;
        s_channels[ENCODER_ID_LEFT].speed_rpm = 0.0f;
        s_channels[ENCODER_ID_LEFT].linear_velocity_m_s = 0.0f;
    }
    else if (id == ENCODER_ID_RIGHT)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        s_channels[ENCODER_ID_RIGHT].last_raw_cnt = 0;
        s_channels[ENCODER_ID_RIGHT].total_ticks = 0;
        s_channels[ENCODER_ID_RIGHT].delta_ticks = 0;
        s_channels[ENCODER_ID_RIGHT].speed_rpm = 0.0f;
        s_channels[ENCODER_ID_RIGHT].linear_velocity_m_s = 0.0f;
    }
}

static const encoder_interface_t s_encoder_interface = {
    .init                = TIM_Encoder_Init,
    .update              = TIM_Encoder_Update,
    .get_total_ticks     = TIM_Encoder_GetTotalTicks,
    .get_delta_ticks     = TIM_Encoder_GetDeltaTicks,
    .get_linear_velocity = TIM_Encoder_GetLinearVelocity,
    .reset               = TIM_Encoder_Reset,
};

const encoder_interface_t* TIM_Encoder_GetInterface(void)
{
    return &s_encoder_interface;
}
