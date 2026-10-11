/**
 * @file task_sensor_acq.h
 * @brief Sensor Acquisition Task (IMU & GPS) for STM32_Mobility.
 */

#ifndef TASK_SENSOR_ACQ_H
#define TASK_SENSOR_ACQ_H

#include "mpu6050_driver.h"
#include "neo6m_driver.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_SensorAcq_Init(void);
bool Task_SensorAcq_GetLatestImu(mpu6050_data_t *imu);
bool Task_SensorAcq_GetLatestGps(neo6m_gps_data_t *gps);
void StartSensorAcqTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SENSOR_ACQ_H */
