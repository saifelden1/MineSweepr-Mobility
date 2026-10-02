/**
 * @file kinematics.c
 * @brief Differential Drive Skid-Steer Forward & Inverse Kinematics Implementation.
 */

#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

static float s_track_gauge_m       = ROBOT_TRACK_GAUGE_M;
static float s_wheel_radius_m      = ROBOT_WHEEL_RADIUS_M;
static float s_max_linear_vel      = ROBOT_MAX_LINEAR_VEL_M_S;
static float s_max_angular_vel     = ROBOT_MAX_ANGULAR_VEL_RAD_S;
static float s_min_deadband_duty   = MOTOR_MIN_DEADBAND_DUTY;

static float clamp_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void Kinematics_Init(const kinematics_config_t *config)
{
    if (config != NULL) {
        if (config->track_gauge_m > 0.01f) {
            s_track_gauge_m = config->track_gauge_m;
        }
        if (config->wheel_radius_m > 0.01f) {
            s_wheel_radius_m = config->wheel_radius_m;
        }
        if (config->max_linear_vel > 0.01f) {
            s_max_linear_vel = config->max_linear_vel;
        }
        if (config->max_angular_vel > 0.01f) {
            s_max_angular_vel = config->max_angular_vel;
        }
        if (config->min_deadband_duty >= 0.0f && config->min_deadband_duty < 0.5f) {
            s_min_deadband_duty = config->min_deadband_duty;
        }
    } else {
        s_track_gauge_m     = ROBOT_TRACK_GAUGE_M;
        s_wheel_radius_m    = ROBOT_WHEEL_RADIUS_M;
        s_max_linear_vel    = ROBOT_MAX_LINEAR_VEL_M_S;
        s_max_angular_vel   = ROBOT_MAX_ANGULAR_VEL_RAD_S;
        s_min_deadband_duty = MOTOR_MIN_DEADBAND_DUTY;
    }
}

void Kinematics_InverseSkidSteer(const chassis_twist_t *twist, float *out_v_left, float *out_v_right)
{
    if (twist == NULL || out_v_left == NULL || out_v_right == NULL) {
        return;
    }

    /* Differential drive skid-steer equations:
     * v_left  = v_x - (omega_z * L / 2)
     * v_right = v_x + (omega_z * L / 2)
     */
    float half_gauge = 0.5f * s_track_gauge_m * twist->angular_z_rad_s;
    *out_v_left  = twist->linear_x_m_s - half_gauge;
    *out_v_right = twist->linear_x_m_s + half_gauge;
}

void Kinematics_Inverse(const chassis_twist_t *twist, wheel_speeds_t *speeds)
{
    if (twist == NULL || speeds == NULL) {
        return;
    }

    Kinematics_InverseSkidSteer(twist, &speeds->left_m_s, &speeds->right_m_s);
    speeds->left_rpm  = Kinematics_VelocityToRpm(speeds->left_m_s);
    speeds->right_rpm = Kinematics_VelocityToRpm(speeds->right_m_s);
}

void Kinematics_Forward(float v_left, float v_right, chassis_twist_t *twist)
{
    if (twist == NULL) {
        return;
    }

    twist->linear_x_m_s    = 0.5f * (v_right + v_left);
    twist->angular_z_rad_s = (v_right - v_left) / s_track_gauge_m;
}

float Kinematics_ApplyDeadbandCompensation(float normalized_duty)
{
    if (fabsf(normalized_duty) <= 0.001f) {
        return 0.0f;
    }

    float sign = (normalized_duty > 0.0f) ? 1.0f : -1.0f;
    float mag  = fabsf(normalized_duty);
    if (mag > 1.0f) mag = 1.0f;

    /* Scaled deadband compensation:
     * D_eff = sgn(D) * [D_min + (1.0 - D_min) * |D|]
     */
    float compensated = s_min_deadband_duty + (1.0f - s_min_deadband_duty) * mag;
    return sign * compensated;
}

void Kinematics_ComputeMotorDuties(const chassis_twist_t *twist, float gyro_z_rad_s, drivetrain_duties_t *out_duties)
{
    if (twist == NULL || out_duties == NULL) {
        return;
    }

    /* 1. Calculate raw perimeter speeds from twist */
    float v_left = 0.0f, v_right = 0.0f;
    Kinematics_InverseSkidSteer(twist, &v_left, &v_right);

    /* 2. Normalize by max linear velocity */
    float d_l = v_left / s_max_linear_vel;
    float d_r = v_right / s_max_linear_vel;

    d_l = clamp_float(d_l, -1.0f, 1.0f);
    d_r = clamp_float(d_r, -1.0f, 1.0f);

    /* 3. Optional IMU yaw damping during commanded straight motion */
    if (fabsf(twist->angular_z_rad_s) < 0.01f && fabsf(twist->linear_x_m_s) > 0.05f) {
        /* Gyro yaw damping: correction = Kp * (0 - measured_gyro_z) */
        float delta_yaw = YAW_DAMPING_KP * (0.0f - gyro_z_rad_s);
        d_l -= delta_yaw;
        d_r += delta_yaw;
        d_l = clamp_float(d_l, -1.0f, 1.0f);
        d_r = clamp_float(d_r, -1.0f, 1.0f);
    }

    /* 4. Apply static friction deadband compensation */
    float comp_l = Kinematics_ApplyDeadbandCompensation(d_l);
    float comp_r = Kinematics_ApplyDeadbandCompensation(d_r);

    /* 5. Populate 4-wheel duty cycle structure */
    out_duties->fl_duty    = comp_l;
    out_duties->rl_duty    = comp_l;
    out_duties->fr_duty    = comp_r;
    out_duties->rr_duty    = comp_r;
    out_duties->left_duty  = comp_l;
    out_duties->right_duty = comp_r;
}

float Kinematics_VelocityToRpm(float velocity_m_s)
{
    float circum = 2.0f * (float)M_PI * s_wheel_radius_m;
    if (circum < 1e-6f) {
        return 0.0f;
    }
    return (velocity_m_s * 60.0f) / circum;
}

float Kinematics_RpmToVelocity(float rpm)
{
    float circum = 2.0f * (float)M_PI * s_wheel_radius_m;
    return (rpm * circum) / 60.0f;
}

float Kinematics_VelocityToTicksPerSec(float velocity_m_s)
{
    float circum = 2.0f * (float)M_PI * s_wheel_radius_m;
    if (circum < 1e-6f) {
        return 0.0f;
    }
    float revs_per_sec = velocity_m_s / circum;
    return revs_per_sec * ROBOT_ENCODER_TICKS_PER_REV;
}
