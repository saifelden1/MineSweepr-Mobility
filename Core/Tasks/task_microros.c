/**
 * @file task_microros.c
 * @brief micro-ROS Client FreeRTOS Task Implementation for STM32_Mobility.
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
    if (twist != NULL)
    {
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
    uint32_t last_hb_tick = 0;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        /* 1. Process incoming serial commands (/cmd_vel) */
        MicroROS_Client_SpinSome(5);

        /* 2. Publish /imu/data @ 50 Hz (20 ms period) */
        if (now - last_imu_tick >= pdMS_TO_TICKS(20))
        {
            last_imu_tick = now;
            mpu6050_data_t imu_raw;
            if (Task_SensorAcq_GetLatestImu(&imu_raw))
            {
                imu_msg_t imu_msg;
                imu_msg.orientation.w = imu_raw.orientation_quat[0];
                imu_msg.orientation.x = imu_raw.orientation_quat[1];
                imu_msg.orientation.y = imu_raw.orientation_quat[2];
                imu_msg.orientation.z = imu_raw.orientation_quat[3];
                imu_msg.angular_velocity.x = imu_raw.gyro_x_rad_s;
                imu_msg.angular_velocity.y = imu_raw.gyro_y_rad_s;
                imu_msg.angular_velocity.z = imu_raw.gyro_z_rad_s;
                imu_msg.linear_acceleration.x = imu_raw.accel_x_m_s2;
                imu_msg.linear_acceleration.y = imu_raw.accel_y_m_s2;
                imu_msg.linear_acceleration.z = imu_raw.accel_z_m_s2;

                MicroROS_Client_PublishImu(&imu_msg);
            }
        }

        /* 3. Publish /gps/fix @ 5 Hz (200 ms period) */
        if (now - last_gps_tick >= pdMS_TO_TICKS(200))
        {
            last_gps_tick = now;
            neo6m_gps_data_t gps_raw;
            if (Task_SensorAcq_GetLatestGps(&gps_raw))
            {
                navsatfix_msg_t gps_msg;
                gps_msg.latitude = gps_raw.latitude;
                gps_msg.longitude = gps_raw.longitude;
                gps_msg.altitude = gps_raw.altitude;
                gps_msg.status = gps_raw.fix_quality;

                MicroROS_Client_PublishGps(&gps_msg);
            }
        }

        /* 4. Publish /stm32_heartbeat @ 1 Hz (1000 ms period) */
        if (now - last_hb_tick >= pdMS_TO_TICKS(1000))
        {
            last_hb_tick = now;
            MicroROS_Client_PublishHeartbeat(true);
        }

        osDelay(5);
    }
}
