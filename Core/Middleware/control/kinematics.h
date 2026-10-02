/**
 * @file kinematics.h
 * @brief Differential Drive Skid-Steer Forward & Inverse Kinematics.
 *
 * Implements differential skid-steer kinematics for Autonomous Minesweeper Rover:
 *   Track Gauge L = 0.55 m
 *   Wheel Radius R = 0.13 m
 *   Max Linear Velocity = 1.0 m/s
 *   Max Angular Velocity = 3.0 rad/s
 *   Minimum Deadband Duty = 0.08 (8%)
 *
 * Equations:
 *   v_left  = v_x - (omega_z * L / 2)
 *   v_right = v_x + (omega_z * L / 2)
 *   v_x     = (v_right + v_left) / 2
 *   omega_z = (v_right - v_left) / L
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "mobility_pin_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ROBOT_TRACK_GAUGE_M
#define ROBOT_TRACK_GAUGE_M         (0.55f)               /**< Distance between wheels L in meters */
#endif

#ifndef ROBOT_WHEEL_RADIUS_M
#define ROBOT_WHEEL_RADIUS_M        (0.13f)               /**< Wheel radius R in meters */
#endif

#define ROBOT_WHEEL_CIRCUMFERENCE_M (0.8168140899333462f) /**< 2 * PI * R */
#define ROBOT_ENCODER_TICKS_PER_REV (1320.0f)             /**< 11 * 4 * 30 */
#define ROBOT_METERS_PER_TICK       (ROBOT_WHEEL_CIRCUMFERENCE_M / ROBOT_ENCODER_TICKS_PER_REV)

#ifndef ROBOT_MAX_LINEAR_VEL_M_S
#define ROBOT_MAX_LINEAR_VEL_M_S    (1.00f)               /**< Max linear speed in m/s */
#endif

#ifndef ROBOT_MAX_ANGULAR_VEL_RAD_S
#define ROBOT_MAX_ANGULAR_VEL_RAD_S (3.00f)               /**< Max angular velocity in rad/s */
#endif

#ifndef MOTOR_MIN_DEADBAND_DUTY
#define MOTOR_MIN_DEADBAND_DUTY     (0.08f)               /**< Minimum duty to overcome stiction */
#endif

#define YAW_DAMPING_KP              (0.05f)               /**< Proportional gyro yaw damping gain */

typedef struct {
    float track_gauge_m;
    float wheel_radius_m;
    float max_linear_vel;
    float max_angular_vel;
    float min_deadband_duty;
} kinematics_config_t;

typedef struct {
    float linear_x_m_s;     /**< Longitudinal linear velocity in m/s */
    float angular_z_rad_s;  /**< Rotational yaw rate in rad/s */
} chassis_twist_t;

typedef struct {
    float left_m_s;         /**< Left wheel perimeter linear velocity in m/s */
    float right_m_s;        /**< Right wheel perimeter linear velocity in m/s */
    float left_rpm;         /**< Left wheel rotational speed in RPM */
    float right_rpm;        /**< Right wheel rotational speed in RPM */
} wheel_speeds_t;

typedef struct {
    float fl_duty;          /**< Front-Left normalized duty [-1.0f .. 1.0f] */
    float rl_duty;          /**< Rear-Left normalized duty [-1.0f .. 1.0f] */
    float fr_duty;          /**< Front-Right normalized duty [-1.0f .. 1.0f] */
    float rr_duty;          /**< Rear-Right normalized duty [-1.0f .. 1.0f] */
    float left_duty;        /**< Combined Left pair normalized duty */
    float right_duty;       /**< Combined Right pair normalized duty */
} drivetrain_duties_t;

/**
 * @brief Initialize kinematics subsystem with robot geometry and limits.
 * @param config Pointer to configuration struct, or NULL for defaults.
 */
void Kinematics_Init(const kinematics_config_t *config);

/**
 * @brief Inverse kinematics: compute target wheel linear speeds from chassis twist request.
 * @param twist Input chassis linear and angular twist.
 * @param speeds Output target wheel linear velocities and RPM.
 */
void Kinematics_Inverse(const chassis_twist_t *twist, wheel_speeds_t *speeds);

/**
 * @brief Simple skid-steer inverse kinematics returning left and right perimeter speeds in m/s.
 * @param twist Input chassis velocity request.
 * @param out_v_left Output left wheel perimeter linear velocity.
 * @param out_v_right Output right wheel perimeter linear velocity.
 */
void Kinematics_InverseSkidSteer(const chassis_twist_t *twist, float *out_v_left, float *out_v_right);

/**
 * @brief Forward kinematics: compute chassis velocities from measured wheel speeds.
 * @param v_left Measured left wheel linear velocity in m/s.
 * @param v_right Measured right wheel linear velocity in m/s.
 * @param twist Output reconstructed chassis linear and angular velocities.
 */
void Kinematics_Forward(float v_left, float v_right, chassis_twist_t *twist);

/**
 * @brief Apply deadband static friction compensation to normalized duty cycle.
 *        If |duty| > 0.001f, scales: sgn(duty) * [deadband + (1 - deadband) * |duty|].
 * @param normalized_duty Input duty [-1.0f .. 1.0f].
 * @return Compensated duty cycle [-1.0f .. 1.0f].
 */
float Kinematics_ApplyDeadbandCompensation(float normalized_duty);

/**
 * @brief Full skid-steer kinematics pipeline: computes duty cycles for all 4 motors
 *        from commanded twist, with deadband compensation and optional IMU gyro Z yaw damping.
 * @param twist Commanded chassis twist (/cmd_vel).
 * @param gyro_z_rad_s Measured IMU gyro Z rate in rad/s (pass 0.0f to disable yaw damping).
 * @param out_duties Destination structure for 4 motor duty cycles.
 */
void Kinematics_ComputeMotorDuties(const chassis_twist_t *twist, float gyro_z_rad_s, drivetrain_duties_t *out_duties);

/**
 * @brief Convert linear wheel velocity in m/s to wheel RPM.
 */
float Kinematics_VelocityToRpm(float velocity_m_s);

/**
 * @brief Convert wheel RPM to linear wheel velocity in m/s.
 */
float Kinematics_RpmToVelocity(float rpm);

/**
 * @brief Convert linear velocity in m/s to encoder ticks per second.
 */
float Kinematics_VelocityToTicksPerSec(float velocity_m_s);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
