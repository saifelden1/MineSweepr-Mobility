/**
 * @file odometry.c
 * @brief Forward Odometry Computation Implementation.
 */

#include "odometry.h"
#include "kinematics.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

static odometry_state_t s_global_odom;

void Odometry_Init(odometry_state_t *odom)
{
    odometry_state_t *target = (odom != NULL) ? odom : &s_global_odom;
    memset(target, 0, sizeof(odometry_state_t));
    target->quat_w = 1.0f; /* Unit quaternion at zero rotation */
}

void Odometry_Reset(odometry_state_t *odom)
{
    Odometry_Init(odom);
}

void Odometry_Update(odometry_state_t *odom, int32_t delta_ticks_left,
                     int32_t delta_ticks_right, float dt_s)
{
    odometry_state_t *target = (odom != NULL) ? odom : &s_global_odom;

    /* Accumulate ticks */
    target->total_ticks_left += delta_ticks_left;
    target->total_ticks_right += delta_ticks_right;

    /* Convert ticks to linear travel distances */
    float dist_left = (float)delta_ticks_left * ROBOT_METERS_PER_TICK;
    float dist_right = (float)delta_ticks_right * ROBOT_METERS_PER_TICK;

    /* Displacement and heading change */
    float delta_s = 0.5f * (dist_right + dist_left);
    float delta_theta = (dist_right - dist_left) / ROBOT_TRACK_GAUGE_M;

    /* Midpoint Runge-Kutta integration */
    float mid_theta = target->yaw_rad + (0.5f * delta_theta);
    target->x_m += delta_s * cosf(mid_theta);
    target->y_m += delta_s * sinf(mid_theta);

    /* Update heading angle normalized to [-PI .. PI] */
    target->yaw_rad += delta_theta;
    target->yaw_rad = atan2f(sinf(target->yaw_rad), cosf(target->yaw_rad));

    /* Synthesize 3D quaternion for planar 2D rotation */
    float half_yaw = 0.5f * target->yaw_rad;
    target->quat_x = 0.0f;
    target->quat_y = 0.0f;
    target->quat_z = sinf(half_yaw);
    target->quat_w = cosf(half_yaw);

    /* Velocity estimation */
    if (dt_s > 1e-5f) {
        target->linear_vel_m_s = delta_s / dt_s;
        target->angular_vel_rad_s = delta_theta / dt_s;
        target->wheel_left_vel_m_s = dist_left / dt_s;
        target->wheel_right_vel_m_s = dist_right / dt_s;
    } else {
        target->linear_vel_m_s = 0.0f;
        target->angular_vel_rad_s = 0.0f;
        target->wheel_left_vel_m_s = 0.0f;
        target->wheel_right_vel_m_s = 0.0f;
    }

    if (odom == NULL) {
        /* Already updated s_global_odom */
    } else if (odom != &s_global_odom) {
        s_global_odom = *odom;
    }
}

void Odometry_GetState(odometry_state_t *out_state)
{
    if (out_state != NULL) {
        *out_state = s_global_odom;
    }
}
