/**
 * @file pid_controller.c
 * @brief Discrete Filtered Velocity PID Controller Implementation.
 */

#include "pid_controller.h"
#include <math.h>

void PID_Init(pid_controller_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_max, float tau, uint16_t arr)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->dt = (dt > 0.0f) ? dt : PID_DEFAULT_DT_S;
    pid->i_max = (i_max > 0.0f) ? i_max : PID_DEFAULT_I_MAX;
    pid->tau = (tau >= 0.0f) ? tau : PID_DEFAULT_TAU_S;
    pid->arr = (arr > 0) ? arr : PID_DEFAULT_ARR;

    float denom = pid->tau + pid->dt;
    pid->alpha = (denom > 0.0f) ? (pid->tau / denom) : 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_derivative = 0.0f;
}

void PID_InitDefault(pid_controller_t *pid)
{
    PID_Init(pid,
             PID_DEFAULT_KP,
             PID_DEFAULT_KI,
             PID_DEFAULT_KD,
             PID_DEFAULT_KFF,
             PID_DEFAULT_DT_S,
             PID_DEFAULT_I_MAX,
             PID_DEFAULT_TAU_S,
             PID_DEFAULT_ARR);
}

void PID_Reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_derivative = 0.0f;
}

float PID_Update(pid_controller_t *pid, float target_v, float measured_v,
                 uint16_t *out_ccr, int8_t *out_dir)
{
    if (pid == NULL) {
        if (out_ccr) *out_ccr = 0;
        if (out_dir) *out_dir = 0;
        return 0.0f;
    }

    float error = target_v - measured_v;

    /* Proportional term */
    float p = pid->kp * error;

    /* Integral term with anti-windup clamping */
    pid->integral += pid->ki * error * pid->dt;
    if (pid->integral > pid->i_max) {
        pid->integral = pid->i_max;
    } else if (pid->integral < -pid->i_max) {
        pid->integral = -pid->i_max;
    }

    /* Filtered derivative term */
    float raw_derivative = (pid->dt > 0.0f) ? ((error - pid->prev_error) / pid->dt) : 0.0f;
    pid->filtered_derivative = (pid->alpha * pid->filtered_derivative) +
                               ((1.0f - pid->alpha) * raw_derivative);
    float d = pid->kd * pid->filtered_derivative;

    /* Feedforward term */
    float ff = pid->kff * target_v;

    /* Total control signal clamped to [-1.0f .. 1.0f] */
    float u = ff + p + pid->integral + d;
    float u_clamped = u;
    if (u_clamped > 1.0f) {
        u_clamped = 1.0f;
    } else if (u_clamped < -1.0f) {
        u_clamped = -1.0f;
    }

    /* Map to ARR compare ticks */
    float abs_u = fabsf(u_clamped);
    uint32_t ccr_val = (uint32_t)roundf(abs_u * (float)pid->arr);
    if (ccr_val > pid->arr) {
        ccr_val = pid->arr;
    }

    int8_t dir = 0;
    if (abs_u < 1e-4f) {
        dir = 0;
    } else if (u_clamped > 0.0f) {
        dir = 1;
    } else {
        dir = -1;
    }

    if (out_ccr != NULL) {
        *out_ccr = (uint16_t)ccr_val;
    }
    if (out_dir != NULL) {
        *out_dir = dir;
    }

    pid->prev_error = error;
    return u_clamped;
}
