/**
 * @file task_microros.h
 * @brief FreeRTOS micro-ROS Communication Task (UART DMA 921600 baud).
 *
 * Runs micro-ROS executor, serializes and publishes /imu/data (50 Hz),
 * /gps/fix (5 Hz), /stm32_heartbeat (1 Hz), and processes /cmd_vel subscriptions.
 */

#ifndef TASK_MICROROS_H
#define TASK_MICROROS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MICROROS_SPIN_PERIOD_MS     (10U)   /**< Fast spin interval */
#define IMU_PUB_PERIOD_MS           (20U)   /**< 50 Hz /imu/data */
#define GPS_PUB_PERIOD_MS           (200U)  /**< 5 Hz /gps/fix */
#define HEARTBEAT_PUB_PERIOD_MS     (1000U) /**< 1 Hz /stm32_heartbeat */

/**
 * @brief Initialize micro-ROS task, transport, and publishers.
 */
void Task_MicroROS_Init(void);

/**
 * @brief FreeRTOS task entry point.
 */
void StartMicroROSTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MICROROS_H */
