/**
 * @file task_microros.c
 * @brief FreeRTOS micro-ROS Communication Task Implementation.
 */

#include "task_microros.h"
#include "task_safety_watchdog.h"
#include "task_motor_control.h"
#include "task_sensor_acq.h"
#include "microros_client.h"
#include "cmsis_os.h"
#include <string.h>

static void on_cmd_vel_received(const twist_msg_t *twist)
{
    if (twist != NULL) {
        Task_SafetyWatchdog_Feed();
        Task_MotorControl_SetTarget(twist->linear.x, twist->angular.z);
    }
}

void Task_MicroROS_Init(void)
{
    MicroROS_Client_Init();
    MicroROS_Client_RegisterCmdVelCallback(on_cmd_vel_received);
}

void StartMicroROSTask(void *argument)
{
    (void)argument;
    Task_MicroROS_Init();

    uint32_t last_imu_tick = 0;
    uint32_t last_gps_tick = 0;
    uint32_t last_heartbeat_tick = 0;

    for (;;) {
        uint32_t current_tick = xTaskGetTickCount();

        /* 1. Spin transport to process incoming serial frames (/cmd_vel) */
        MicroROS_Client_SpinSome(5);

        /* 2. Publish /imu/data at 50 Hz (every 20 ms) */
        if ((current_tick - last_imu_tick) * portTICK_PERIOD_MS >= IMU_PUB_PERIOD_MS) {
            last_imu_tick = current_tick;

            imu_data_t imu_data;
            if (Task_SensorAcq_GetLatestImu(&imu_data)) {
                imu_msg_t imu_msg;
                memset(&imu_msg, 0, sizeof(imu_msg));
                imu_msg.sec = current_tick / 1000U;
                imu_msg.nanosec = (current_tick % 1000U) * 1000000U;
                strncpy(imu_msg.frame_id, "imu_link", sizeof(imu_msg.frame_id) - 1);

                imu_msg.orientation.w = imu_data.orientation_quat[0];
                imu_msg.orientation.x = imu_data.orientation_quat[1];
                imu_msg.orientation.y = imu_data.orientation_quat[2];
                imu_msg.orientation.z = imu_data.orientation_quat[3];

                imu_msg.angular_velocity.x = imu_data.gyro_x_rad_s;
                imu_msg.angular_velocity.y = imu_data.gyro_y_rad_s;
                imu_msg.angular_velocity.z = imu_data.gyro_z_rad_s;

                imu_msg.linear_acceleration.x = imu_data.accel_x_m_s2;
                imu_msg.linear_acceleration.y = imu_data.accel_y_m_s2;
                imu_msg.linear_acceleration.z = imu_data.accel_z_m_s2;

                MicroROS_Client_PublishImu(&imu_msg);
            }
        }

        /* 3. Publish /gps/fix at 5 Hz (every 200 ms) */
        if ((current_tick - last_gps_tick) * portTICK_PERIOD_MS >= GPS_PUB_PERIOD_MS) {
            last_gps_tick = current_tick;

            gps_data_t gps_data;
            if (Task_SensorAcq_GetLatestGps(&gps_data)) {
                navsatfix_msg_t gps_msg;
                memset(&gps_msg, 0, sizeof(gps_msg));
                gps_msg.sec = current_tick / 1000U;
                gps_msg.nanosec = (current_tick % 1000U) * 1000000U;
                strncpy(gps_msg.frame_id, "gps_link", sizeof(gps_msg.frame_id) - 1);

                gps_msg.status = gps_data.fix_valid ? 0 : -1;
                gps_msg.service = 1; /* GPS service */
                gps_msg.latitude = gps_data.latitude_deg;
                gps_msg.longitude = gps_data.longitude_deg;
                gps_msg.altitude = (double)gps_data.altitude_m;

                MicroROS_Client_PublishGps(&gps_msg);
            }
        }

        /* 4. Publish /stm32_heartbeat at 1 Hz (every 1000 ms) */
        if ((current_tick - last_heartbeat_tick) * portTICK_PERIOD_MS >= HEARTBEAT_PUB_PERIOD_MS) {
            last_heartbeat_tick = current_tick;

            bool_msg_t hb_msg;
            hb_msg.data = true;
            MicroROS_Client_PublishHeartbeat(&hb_msg);
        }

        osDelay(MICROROS_SPIN_PERIOD_MS);
    }
}
