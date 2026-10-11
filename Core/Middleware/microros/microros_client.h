/**
 * @file microros_client.h
 * @brief micro-ROS Client & Topic Publisher/Subscriber Manager for STM32_Mobility.
 *
 * Manages serialization, subscription, and dispatch for ROS 2 topics:
 *   Subscribers:
 *     - /cmd_vel         (0x01, geometry_msgs/msg/Twist)
 *   Publishers:
 *     - /imu/data        (0x02, 50 Hz, sensor_msgs/msg/Imu)
 *     - /gps/fix         (0x03, 5 Hz,  sensor_msgs/msg/NavSatFix)
 *     - /stm32_heartbeat (0x04, 1 Hz,  std_msgs/msg/Bool)
 */

#ifndef MICROROS_CLIENT_H
#define MICROROS_CLIENT_H

#include "microros_msgs.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cmd_vel_callback_t)(const twist_msg_t *twist);

typedef struct {
    uint32_t imu_published_count;
    uint32_t gps_published_count;
    uint32_t heartbeat_published_count;
    uint32_t cmd_vel_received_count;
    uint32_t framing_error_count;
    bool agent_connected;
} microros_mobility_stats_t;

/**
 * @brief Initialize micro-ROS client, transport, publishers, and subscribers.
 * @return true on success.
 */
bool MicroROS_Client_Init(void);

/**
 * @brief Register application callback for incoming /cmd_vel messages.
 * @param cb Callback function receiving linear & angular velocity.
 */
void MicroROS_Client_RegisterCmdVelCallback(cmd_vel_callback_t cb);

/**
 * @brief Process incoming packets from UART DMA circular buffer.
 * @param timeout_ms Max spin duration in milliseconds.
 */
void MicroROS_Client_SpinSome(uint32_t timeout_ms);

/**
 * @brief Publish /imu/data topic.
 * @param msg Pointer to imu_msg_t payload.
 * @return true on success.
 */
bool MicroROS_Client_PublishImu(const imu_msg_t *msg);

/**
 * @brief Publish /gps/fix topic.
 * @param msg Pointer to navsatfix_msg_t payload.
 * @return true on success.
 */
bool MicroROS_Client_PublishGps(const navsatfix_msg_t *msg);

/**
 * @brief Publish /stm32_heartbeat topic.
 * @param state Boolean heartbeat flag.
 * @return true on success.
 */
bool MicroROS_Client_PublishHeartbeat(bool state);

/**
 * @brief Get client operational statistics.
 */
void MicroROS_Client_GetStats(microros_mobility_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_CLIENT_H */
