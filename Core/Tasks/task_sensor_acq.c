/**
 * @file task_sensor_acq.c
 * @brief FreeRTOS Sensor Acquisition Task Implementation.
 */

#include "task_sensor_acq.h"
#include "mpu6050_driver.h"
#include "neo6m_driver.h"
#include "cmsis_os.h"
#include <string.h>

static imu_data_t s_latest_imu;
static gps_data_t s_latest_gps;
static bool s_has_imu_data = false;
static bool s_has_gps_fix = false;

void Task_SensorAcq_Init(void)
{
    memset(&s_latest_imu, 0, sizeof(s_latest_imu));
    memset(&s_latest_gps, 0, sizeof(s_latest_gps));
    s_has_imu_data = false;
    s_has_gps_fix = false;
}

bool Task_SensorAcq_GetLatestImu(imu_data_t *out_imu)
{
    if (out_imu != NULL && s_has_imu_data) {
        *out_imu = s_latest_imu;
        return true;
    }
    return false;
}

float Task_SensorAcq_GetGyroZ(void)
{
    if (s_has_imu_data) {
        return s_latest_imu.gyro_z_rad_s;
    }
    return 0.0f;
}

bool Task_SensorAcq_GetLatestGps(gps_data_t *out_gps)
{
    if (out_gps != NULL && s_has_gps_fix) {
        *out_gps = s_latest_gps;
        return true;
    }
    return false;
}

void StartSensorAcqTask(void *argument)
{
    (void)argument;
    Task_SensorAcq_Init();

    uint32_t cycle_count = 0;

    for (;;) {
        /* 1. Sample 6-DOF IMU at 50 Hz */
        if (MPU6050_ReadCalibrated(&s_latest_imu)) {
            s_has_imu_data = true;
        }

        /* 2. Sample GPS fix at 5 Hz (decimation of 10 cycles @ 20 ms) */
        cycle_count++;
        if (cycle_count >= GPS_SAMPLE_DECIMATION) {
            cycle_count = 0;
            if (NEO6M_GetFix(&s_latest_gps)) {
                s_has_gps_fix = true;
            }
        }

        osDelay(SENSOR_ACQ_PERIOD_MS);
    }
}
