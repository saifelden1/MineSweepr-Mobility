/**
 * @file task_motor_control.c
 * @brief FreeRTOS Open-Loop Differential Skid-Steer Motor Control Task Implementation.
 */

#include "task_motor_control.h"
#include "task_safety_watchdog.h"
#include "task_sensor_acq.h"
#include "kinematics.h"
#include "cytron_mdd10a_driver.h"
#include "cmsis_os.h"

static volatile float s_target_linear_x = 0.0f;
static volatile float s_target_angular_z = 0.0f;

void Task_MotorControl_Init(void)
{
    Kinematics_Init(NULL);
    s_target_linear_x = 0.0f;
    s_target_angular_z = 0.0f;
}

void Task_MotorControl_SetTarget(float linear_x, float angular_z)
{
    s_target_linear_x = linear_x;
    s_target_angular_z = angular_z;
}

void Task_MotorControl_GetTarget(float *out_linear_x, float *out_angular_z)
{
    if (out_linear_x != NULL) *out_linear_x = s_target_linear_x;
    if (out_angular_z != NULL) *out_angular_z = s_target_angular_z;
}

void StartMotorControlTask(void *argument)
{
    (void)argument;
    Task_MotorControl_Init();

    for (;;) {
        /* 1. Safety Watchdog Interlock Check */
        if (Task_SafetyWatchdog_IsTripped()) {
            Cytron_MDD10A_EmergencyStopAll();
        } else {
            /* 2. Assemble commanded twist from /cmd_vel */
            chassis_twist_t twist;
            twist.linear_x_m_s    = s_target_linear_x;
            twist.angular_z_rad_s = s_target_angular_z;

            /* 3. Query latest gyro Z rate for yaw damping */
            float gyro_z = Task_SensorAcq_GetGyroZ();

            /* 4. Compute 4-motor duty cycles via skid-steer kinematics with deadband compensation */
            drivetrain_duties_t duties;
            Kinematics_ComputeMotorDuties(&twist, gyro_z, &duties);

            /* 5. Command all 4 motors via Cytron MDD10A drivers */
            Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_FL, duties.fl_duty);
            Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_RL, duties.rl_duty);
            Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_FR, duties.fr_duty);
            Cytron_MDD10A_SetSpeedNorm(MOTOR_ID_RR, duties.rr_duty);
        }

        osDelay(MOTOR_CONTROL_PERIOD_MS);
    }
}
