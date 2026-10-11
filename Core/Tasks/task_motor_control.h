/**
 * @file task_motor_control.h
 * @brief Skid-Steer Motor Kinematics & Control Task for STM32_Mobility.
 */

#ifndef TASK_MOTOR_CONTROL_H
#define TASK_MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_MotorControl_Init(void);
void Task_MotorControl_SetTarget(float linear_x, float angular_z);
bool Task_MotorControl_IsMotionActive(void);
void StartMotorControlTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_CONTROL_H */
