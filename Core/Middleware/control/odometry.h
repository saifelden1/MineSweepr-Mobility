/**
 * @file odometry.h
 * @brief Forward Odometry Computation & Midpoint Integration.
 *
 * Tracks 2D pose (x, y, theta), 3D quaternion orientation, and twist
 * velocities from wheel encoder delta pulses.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 2D Pose */
    float x_m;                  /**< X position in odom frame (meters) */
    float y_m;                  /**< Y position in odom frame (meters) */
    float yaw_rad;              /**< Heading angle in radians [-pi .. pi] */

    /* 3D Quaternion [x, y, z, w] */
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;

    /* Chassis Twist */
    float linear_vel_m_s;       /**< Longitudinal linear velocity in m/s */
    float angular_vel_rad_s;    /**< Yaw angular velocity in rad/s */

    /* Wheel Linear Velocities */
    float wheel_left_vel_m_s;
    float wheel_right_vel_m_s;

    /* Cumulative Encoder Metrics */
    int64_t total_ticks_left;
    int64_t total_ticks_right;
} odometry_state_t;

/**
 * @brief Initialize odometry state to origin (0, 0, 0).
 */
void Odometry_Init(odometry_state_t *odom);

/**
 * @brief Reset odometry state to origin.
 */
void Odometry_Reset(odometry_state_t *odom);

/**
 * @brief Update odometry using encoder tick deltas and elapsed time.
 * @param odom Pointer to odometry state structure.
 * @param delta_ticks_left Incremental ticks from left encoder.
 * @param delta_ticks_right Incremental ticks from right encoder.
 * @param dt_s Sampling interval in seconds (e.g. 0.02 for 50 Hz).
 */
void Odometry_Update(odometry_state_t *odom, int32_t delta_ticks_left,
                     int32_t delta_ticks_right, float dt_s);

/**
 * @brief Get copy of current odometry state (thread-safe).
 */
void Odometry_GetState(odometry_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
