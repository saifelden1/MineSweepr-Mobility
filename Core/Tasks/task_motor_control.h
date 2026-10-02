/**
 * @file task_motor_control.h
 * @brief FreeRTOS Open-Loop Differential Skid-Steer Motor Control Task (50 Hz / 20 ms).
 *
 * Drives 4 independent DC motors via Cytron MDD10A drivers based on /cmd_vel
 * with differential skid-steer kinematics, deadband friction compensation,
 * optional IMU gyro Z yaw damping, and safety watchdog interlock.
 */

#ifndef TASK_MOTOR_CONTROL_H
#define TASK_MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONTROL_PERIOD_MS     (20U)   /**< 50 Hz execution period */

/**
 * @brief Initialize motor control task and kinematics subsystem.
 */
void Task_MotorControl_Init(void);

/**
 * @brief Set target chassis velocities from micro-ROS /cmd_vel.
 * @param linear_x Desired forward linear velocity in m/s.
 * @param angular_z Desired rotational velocity in rad/s.
 */
void Task_MotorControl_SetTarget(float linear_x, float angular_z);

/**
 * @brief Retrieve currently commanded target velocities.
 * @param out_linear_x Pointer to destination for linear velocity.
 * @param out_angular_z Pointer to destination for angular velocity.
 */
void Task_MotorControl_GetTarget(float *out_linear_x, float *out_angular_z);

/**
 * @brief FreeRTOS task entry point.
 */
void StartMotorControlTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_CONTROL_H */
