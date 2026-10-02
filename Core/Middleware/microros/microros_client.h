/**
 * @file microros_client.h
 * @brief micro-ROS Client & Topic Publisher/Subscriber Manager.
 *
 * Manages serialization, subscription, and dispatch for ROS 2 topics:
 *   Publishers:
 *     - /imu/data         (50 Hz, sensor_msgs/msg/Imu)
 *     - /gps/fix          (5 Hz,  sensor_msgs/msg/NavSatFix)
 *     - /stm32_heartbeat  (1 Hz,  std_msgs/msg/Bool)
 *     - /odom             (Legacy, nav_msgs/msg/Odometry)
 *   Subscriber:
 *     - /cmd_vel          (20 Hz, geometry_msgs/msg/Twist)
 */

#ifndef MICROROS_CLIENT_H
#define MICROROS_CLIENT_H

#include "microros_msgs.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOPIC_ID_ODOM           (0x01U)
#define TOPIC_ID_IMU            (0x02U)
#define TOPIC_ID_GPS            (0x03U)
#define TOPIC_ID_CMD_VEL        (0x04U)
#define TOPIC_ID_HEARTBEAT      (0x05U)

typedef void (*cmd_vel_callback_t)(const twist_msg_t *twist);

typedef struct {
    uint32_t odom_published_count;
    uint32_t imu_published_count;
    uint32_t gps_published_count;
    uint32_t heartbeat_published_count;
    uint32_t cmd_vel_received_count;
    uint32_t framing_error_count;
    bool agent_connected;
} microros_stats_t;

/**
 * @brief Initialize micro-ROS client, publishers, and subscribers.
 * @return true on success.
 */
bool MicroROS_Client_Init(void);

/**
 * @brief Register application callback for incoming /cmd_vel messages.
 */
void MicroROS_Client_RegisterCmdVelCallback(cmd_vel_callback_t cb);

/**
 * @brief Process incoming packets from UART DMA circular buffer.
 * @param timeout_ms Max spin duration in milliseconds.
 */
void MicroROS_Client_SpinSome(uint32_t timeout_ms);

/**
 * @brief Publish /odom topic message (Legacy 50 Hz).
 */
bool MicroROS_Client_PublishOdom(const odometry_msg_t *odom);

/**
 * @brief Publish /imu/data topic message (50 Hz).
 */
bool MicroROS_Client_PublishImu(const imu_msg_t *imu);

/**
 * @brief Publish /gps/fix topic message (5 Hz).
 */
bool MicroROS_Client_PublishGps(const navsatfix_msg_t *gps);

/**
 * @brief Publish /stm32_heartbeat topic message (1 Hz std_msgs/msg/Bool).
 */
bool MicroROS_Client_PublishHeartbeat(const bool_msg_t *heartbeat);

/**
 * @brief Query current client statistics and health.
 */
void MicroROS_Client_GetStats(microros_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_CLIENT_H */
