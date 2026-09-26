/**
 * @file tim_encoder_driver.h
 * @brief Concrete BSP Driver for Dual Quadrature Encoders via TIM2 and TIM3.
 *
 * Implements encoder_interface_t using TIM2 (PA0/PA1, 32-bit hardware counter)
 * and TIM3 (PA6/PA7, 16-bit hardware counter with two's complement rollover).
 */

#ifndef TIM_ENCODER_DRIVER_H
#define TIM_ENCODER_DRIVER_H

#include "interfaces/encoder_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_CPR                 (11U)
#define ENCODER_QUAD_MULTIPLIER     (4U)
#define ENCODER_GEAR_RATIO          (30U)
#define ENCODER_TICKS_PER_REV_F     (1320.0f)
#define WHEEL_RADIUS_M_F            (0.13f)
#define WHEEL_CIRCUMFERENCE_M_F     (0.8168140899333462f)  /**< 2 * PI * 0.13 */
#define METERS_PER_TICK_F           (WHEEL_CIRCUMFERENCE_M_F / ENCODER_TICKS_PER_REV_F) /**< ~0.000618802 m */

/**
 * @brief Initialize hardware timers (TIM2 & TIM3) in Encoder Mode.
 * @return true on success.
 */
bool TIM_Encoder_Init(void);

/**
 * @brief Sample hardware counters and compute delta ticks and linear velocities.
 * @param delta_time_us Elapsed sampling interval in microseconds.
 */
void TIM_Encoder_Update(uint32_t delta_time_us);

/**
 * @brief Retrieve accumulated total ticks for specified encoder channel.
 * @param id ENCODER_ID_LEFT or ENCODER_ID_RIGHT.
 * @return Monotonically accumulated 64-bit tick count.
 */
int64_t TIM_Encoder_GetTotalTicks(encoder_id_t id);

/**
 * @brief Retrieve delta ticks elapsed during previous update interval.
 * @param id ENCODER_ID_LEFT or ENCODER_ID_RIGHT.
 * @return Signed 32-bit tick delta.
 */
int32_t TIM_Encoder_GetDeltaTicks(encoder_id_t id);

/**
 * @brief Retrieve linear velocity of wheel surface in m/s.
 * @param id ENCODER_ID_LEFT or ENCODER_ID_RIGHT.
 * @return Linear velocity in m/s.
 */
float TIM_Encoder_GetLinearVelocity(encoder_id_t id);

/**
 * @brief Reset encoder counter and accumulated metrics for specified channel.
 * @param id ENCODER_ID_LEFT or ENCODER_ID_RIGHT.
 */
void TIM_Encoder_Reset(encoder_id_t id);

/**
 * @brief Get the encoder_interface_t function pointer table.
 * @return Pointer to encoder_interface_t instance.
 */
const encoder_interface_t* TIM_Encoder_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* TIM_ENCODER_DRIVER_H */
