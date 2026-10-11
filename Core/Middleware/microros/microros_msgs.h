/**
 * @file microros_msgs.h
 * @brief ROS 2 Message Definitions for STM32_Mobility micro-ROS Client.
 *
 * Defines standard message structures for:
 *   - geometry_msgs/msg/Twist    (/cmd_vel)
 *   - sensor_msgs/msg/Imu        (/imu/data)
 *   - sensor_msgs/msg/NavSatFix  (/gps/fix)
 *   - std_msgs/msg/Bool          (/stm32_heartbeat)
 */

#ifndef MICROROS_MSGS_H
#define MICROROS_MSGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* 1. Topic Identifiers for STM32_Mobility                                   */
/* ------------------------------------------------------------------------- */
#define TOPIC_ID_CMD_VEL          (0x01U) /**< Sub: /cmd_vel (Twist @ 20 Hz) */
#define TOPIC_ID_IMU_DATA         (0x02U) /**< Pub: /imu/data (Imu @ 50 Hz) */
#define TOPIC_ID_GPS_FIX          (0x03U) /**< Pub: /gps/fix (NavSatFix @ 5 Hz) */
#define TOPIC_ID_STM32_HEARTBEAT  (0x04U) /**< Pub: /stm32_heartbeat (Bool @ 1 Hz) */

/* ------------------------------------------------------------------------- */
/* 2. Message Payload Structures                                             */
/* ------------------------------------------------------------------------- */
#pragma pack(push, 1)

/** Vector3 structure */
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

/** Quaternion structure */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} quaternion_t;

/** geometry_msgs/msg/Twist */
typedef struct {
    vector3_t linear;
    vector3_t angular;
} twist_msg_t;

/** sensor_msgs/msg/Imu */
typedef struct {
    quaternion_t orientation;
    vector3_t    angular_velocity;
    vector3_t    linear_acceleration;
} imu_msg_t;

/** sensor_msgs/msg/NavSatFix */
typedef struct {
    double  latitude;
    double  longitude;
    double  altitude;
    int8_t  status;
} navsatfix_msg_t;

/** std_msgs/msg/Bool */
typedef struct {
    bool data;
} bool_msg_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_MSGS_H */
