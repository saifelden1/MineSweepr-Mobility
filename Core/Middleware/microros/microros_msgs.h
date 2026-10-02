/**
 * @file microros_msgs.h
 * @brief ROS 2 Message Definitions for Autonomous Minesweeper micro-ROS Client.
 *
 * Defines C representations for:
 *   - geometry_msgs/msg/Twist (/cmd_vel)
 *   - nav_msgs/msg/Odometry (/odom)
 *   - sensor_msgs/msg/Imu (/imu/data)
 *   - sensor_msgs/msg/NavSatFix (/gps/fix)
 *   - std_msgs/msg/Bool (/stm32_heartbeat)
 */

#ifndef MICROROS_MSGS_H
#define MICROROS_MSGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* 1. geometry_msgs/msg/Twist                                                */
/* ------------------------------------------------------------------------- */
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    vector3_t linear;
    vector3_t angular;
} twist_msg_t;

/* ------------------------------------------------------------------------- */
/* 2. nav_msgs/msg/Odometry                                                  */
/* ------------------------------------------------------------------------- */
typedef struct {
    float x;
    float y;
    float z;
} point_t;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} quaternion_t;

typedef struct {
    point_t position;
    quaternion_t orientation;
} pose_t;

typedef struct {
    uint32_t sec;
    uint32_t nanosec;
    char frame_id[16];
    char child_frame_id[16];
    pose_t pose;
    float pose_covariance[36];
    twist_msg_t twist;
    float twist_covariance[36];
} odometry_msg_t;

/* ------------------------------------------------------------------------- */
/* 3. sensor_msgs/msg/Imu                                                    */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint32_t sec;
    uint32_t nanosec;
    char frame_id[16];
    quaternion_t orientation;
    float orientation_covariance[9];
    vector3_t angular_velocity;
    float angular_velocity_covariance[9];
    vector3_t linear_acceleration;
    float linear_acceleration_covariance[9];
} imu_msg_t;

/* ------------------------------------------------------------------------- */
/* 4. sensor_msgs/msg/NavSatFix                                              */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint32_t sec;
    uint32_t nanosec;
    char frame_id[16];
    int8_t status;
    uint16_t service;
    double latitude;
    double longitude;
    double altitude;
    double position_covariance[9];
    uint8_t position_covariance_type;
} navsatfix_msg_t;

/* ------------------------------------------------------------------------- */
/* 5. std_msgs/msg/Bool                                                      */
/* ------------------------------------------------------------------------- */
typedef struct {
    bool data;
} bool_msg_t;

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_MSGS_H */
