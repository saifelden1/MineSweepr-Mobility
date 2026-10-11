/**
 * @file task_microros.h
 * @brief micro-ROS Client FreeRTOS Task for STM32_Mobility.
 */

#ifndef TASK_MICROROS_H
#define TASK_MICROROS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_MicroROS_Init(void);
void StartMicroROSTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MICROROS_H */
