/**
 * @file mpu6050_driver.h
 * @brief MPU-6050 6-DOF IMU Driver for STM32_Mobility over I2C2.
 */

#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "mobility_pin_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_x_m_s2;
    float accel_y_m_s2;
    float accel_z_m_s2;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    float orientation_quat[4]; /**< w, x, y, z */
} mpu6050_data_t;

/**
 * @brief Initialize MPU-6050 sensor on I2C2.
 * @param hi2c Pointer to I2C_HandleTypeDef (e.g. &hi2c2).
 * @return true on success.
 */
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read 6-DOF data from MPU-6050.
 * @param data Output data structure.
 * @return true on success.
 */
bool MPU6050_ReadData(mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_DRIVER_H */
