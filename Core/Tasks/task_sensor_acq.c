/**
 * @file task_sensor_acq.c
 * @brief Sensor Acquisition Task Implementation (IMU & GPS) for STM32_Mobility.
 */

#include "task_sensor_acq.h"
#include "cmsis_os.h"
#include <string.h>

static mpu6050_data_t s_latest_imu;
static bool s_has_imu = false;

void Task_SensorAcq_Init(void)
{
    memset(&s_latest_imu, 0, sizeof(s_latest_imu));
    s_has_imu = false;
}

bool Task_SensorAcq_GetLatestImu(mpu6050_data_t *imu)
{
    if (imu != NULL && s_has_imu)
    {
        memcpy(imu, &s_latest_imu, sizeof(mpu6050_data_t));
        return true;
    }
    return false;
}

bool Task_SensorAcq_GetLatestGps(neo6m_gps_data_t *gps)
{
    return NEO6M_GetData(gps);
}

void StartSensorAcqTask(void *argument)
{
    (void)argument;
    Task_SensorAcq_Init();

    for (;;)
    {
        /* Periodically drain USART2 DMA circular buffer to update GPS fix */
        NEO6M_ProcessDmaBuffer();

        /* Sample MPU-6050 6-DOF IMU */
        mpu6050_data_t imu_buf;
        if (MPU6050_ReadData(&imu_buf))
        {
            memcpy(&s_latest_imu, &imu_buf, sizeof(mpu6050_data_t));
            s_has_imu = true;
        }

        osDelay(20); // 50 Hz sampling rate
    }
}
