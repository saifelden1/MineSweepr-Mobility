/**
 * @file task_motor_control.c
 * @brief Skid-Steer Motor Kinematics & Control Task Implementation.
 */

#include "task_motor_control.h"
#include "task_safety_watchdog.h"
#include "task_sensor_acq.h"
#include "cytron_mdd10a_driver.h"
#include "mobility_pin_config.h"
#include "cmsis_os.h"
#include <math.h>

static float s_target_linear_x = 0.0f;
static float s_target_angular_z = 0.0f;

void Task_MotorControl_Init(void)
{
    s_target_linear_x = 0.0f;
    s_target_angular_z = 0.0f;
}

void Task_MotorControl_SetTarget(float linear_x, float angular_z)
{
    s_target_linear_x = linear_x;
    s_target_angular_z = angular_z;
}

static float apply_deadband(float duty)
{
    if (fabsf(duty) <= 0.001f)
    {
        return 0.0f;
    }
    float sign = (duty > 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(duty);
    if (mag > 1.0f) mag = 1.0f;
    return sign * (MOTOR_MIN_DEADBAND_DUTY + (1.0f - MOTOR_MIN_DEADBAND_DUTY) * mag);
}

void StartMotorControlTask(void *argument)
{
    (void)argument;
    Task_MotorControl_Init();

    for (;;)
    {
        if (Task_SafetyWatchdog_IsTripped())
        {
            Cytron_MDD10A_BrakeAll();
        }
        else
        {
            /* Closed-loop gyro heading stabilization when driving straight in loose sand */
            float effective_angular_z = s_target_angular_z;
            if (fabsf(s_target_angular_z) < 0.05f && fabsf(s_target_linear_x) > 0.05f)
            {
                mpu6050_data_t imu_data;
                if (Task_SensorAcq_GetLatestImu(&imu_data))
                {
                    float yaw_correction = -0.15f * imu_data.gyro_z_rad_s;
                    if (yaw_correction > 0.4f)  yaw_correction = 0.4f;
                    if (yaw_correction < -0.4f) yaw_correction = -0.4f;
                    effective_angular_z += yaw_correction;
                }
            }

            /* Inverse skid-steer kinematics:
             * v_left  = v_x - (omega_z * L / 2)
             * v_right = v_x + (omega_z * L / 2)
             */
            float half_gauge = 0.5f * ROBOT_TRACK_GAUGE_M * effective_angular_z;
            float v_left  = s_target_linear_x - half_gauge;
            float v_right = s_target_linear_x + half_gauge;

            /* Normalize by max linear velocity */
            float d_left  = v_left / ROBOT_MAX_LINEAR_VEL_M_S;
            float d_right = v_right / ROBOT_MAX_LINEAR_VEL_M_S;

            if (d_left > 1.0f)   d_left = 1.0f;
            if (d_left < -1.0f)  d_left = -1.0f;
            if (d_right > 1.0f)  d_right = 1.0f;
            if (d_right < -1.0f) d_right = -1.0f;

            /* Apply deadband static friction compensation */
            float comp_left  = apply_deadband(d_left);
            float comp_right = apply_deadband(d_right);

            /* Assign to 4 motors */
            Cytron_MDD10A_SetMotors(comp_left, comp_left, comp_right, comp_right);
        }

        osDelay(20); // 50 Hz control rate
    }
}
