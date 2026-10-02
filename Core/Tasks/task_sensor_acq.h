/**
 * @file task_sensor_acq.h
 * @brief FreeRTOS Sensor Acquisition Task (50 Hz IMU + 5 Hz GPS).
 *
 * Samples 6-DOF MPU-6050 IMU over configured I2C Fast Mode at 50 Hz and
 * queries NEO-6M GPS receiver over USART2 at 5 Hz.
 */

#ifndef TASK_SENSOR_ACQ_H
#define TASK_SENSOR_ACQ_H

#include "interfaces/imu_interface.h"
#include "interfaces/gps_interface.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_ACQ_PERIOD_MS        (20U)   /**< 50 Hz primary IMU sampling rate */
#define GPS_SAMPLE_DECIMATION       (10U)   /**< 50 Hz / 10 = 5 Hz GPS query rate */

/**
 * @brief Initialize sensor acquisition task and drivers.
 */
void Task_SensorAcq_Init(void);

/**
 * @brief Retrieve most recent calibrated IMU data.
 * @param out_imu Destination buffer for IMU telemetry.
 * @return true if fresh data was acquired.
 */
bool Task_SensorAcq_GetLatestImu(imu_data_t *out_imu);

/**
 * @brief Retrieve most recent calibrated gyro Z angular rate in rad/s.
 * @return Gyro Z in rad/s (0.0f if not yet acquired).
 */
float Task_SensorAcq_GetGyroZ(void);

/**
 * @brief Retrieve most recent GPS fix data.
 * @param out_gps Destination buffer for GPS fix telemetry.
 * @return true if valid GPS fix is available.
 */
bool Task_SensorAcq_GetLatestGps(gps_data_t *out_gps);

/**
 * @brief FreeRTOS task entry point.
 */
void StartSensorAcqTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SENSOR_ACQ_H */
