/**
 * @file pid_controller.h
 * @brief Discrete Filtered Velocity PID Controller with Anti-Windup.
 *
 * Implements 50 Hz discrete PID velocity controller with feedforward,
 * anti-windup clamping to i_max, low-pass filtered derivative, and
 * PWM ARR=4799 mapping.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEFAULT_KP           (1.2f)
#define PID_DEFAULT_KI           (0.8f)
#define PID_DEFAULT_KD           (0.05f)
#define PID_DEFAULT_KFF          (0.5f)
#define PID_DEFAULT_DT_S         (0.02f)     /**< 50 Hz loop period (20 ms) */
#define PID_DEFAULT_I_MAX        (0.5f)      /**< Anti-windup integrator ceiling */
#define PID_DEFAULT_TAU_S        (0.01f)     /**< Derivative filter time constant */
#define PID_DEFAULT_ARR          (4799U)     /**< TIM4 ARR duty resolution */

typedef struct {
    float kp;
    float ki;
    float kd;
    float kff;
    float dt;
    float i_max;
    float tau;
    float alpha;
    uint16_t arr;

    /* Runtime State */
    float integral;
    float prev_error;
    float filtered_derivative;
} pid_controller_t;

/**
 * @brief Initialize PID controller with custom gains and limits.
 */
void PID_Init(pid_controller_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_max, float tau, uint16_t arr);

/**
 * @brief Initialize PID controller with project default constants.
 */
void PID_InitDefault(pid_controller_t *pid);

/**
 * @brief Reset internal PID state (integrator, derivative filter, previous error).
 */
void PID_Reset(pid_controller_t *pid);

/**
 * @brief Compute PID control update.
 * @param pid Pointer to controller instance.
 * @param target_v Desired linear velocity in m/s.
 * @param measured_v Actual measured linear velocity in m/s.
 * @param out_ccr Pointer to store output PWM duty in timer compare ticks [0 .. ARR].
 * @param out_dir Pointer to store direction (+1 forward, -1 reverse, 0 brake).
 * @return Normalized control signal u in [-1.0f .. 1.0f].
 */
float PID_Update(pid_controller_t *pid, float target_v, float measured_v,
                 uint16_t *out_ccr, int8_t *out_dir);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
