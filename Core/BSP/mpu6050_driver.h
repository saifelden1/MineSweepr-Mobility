/**
 * @file mpu6050_driver.h
 * @brief Concrete BSP Driver for MPU-6050 6-DOF IMU over I2C1 Fast Mode.
 *
 * Implements imu_interface_t with DLPF Mode 3 (44 Hz / 42 Hz), 50 Hz burst reading,
 * calibration routines, and quaternion synthesis.
 */

#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "interfaces/imu_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR            (0x68 << 1) /**< 7-bit 0x68 shifted to 8-bit */
#define MPU6050_WHO_AM_I_VAL        (0x68)
#define MPU6500_WHO_AM_I_VAL        (0x70)

/* Register Map */
#define MPU6050_REG_SMPLRT_DIV      (0x19)
#define MPU6050_REG_CONFIG          (0x1A)
#define MPU6050_REG_GYRO_CONFIG     (0x1B)
#define MPU6050_REG_ACCEL_CONFIG    (0x1C)
#define MPU6050_REG_ACCEL_XOUT_H    (0x3B)
#define MPU6050_REG_PWR_MGMT_1      (0x6B)
#define MPU6050_REG_WHO_AM_I        (0x75)

/* Scaling Factors */
#define MPU6050_ACCEL_SENS_2G       (16384.0f)     /**< LSB / g */
#define MPU6050_GYRO_SENS_2000DPS   (16.4f)        /**< LSB / (deg/s) */
#define GRAVITY_MSS                 (9.80665f)     /**< Standard gravity in m/s^2 */
#define DEG_TO_RAD                  (0.017453292519943295f) /**< PI / 180 */

/**
 * @brief Initialize MPU-6050 hardware over I2C1 (wake up, DLPF 3, +-2g, +-2000 dps).
 * @return true on success, false if device fails to respond or WHO_AM_I mismatch.
 */
bool MPU6050_Init(void);

/**
 * @brief Read raw 14-byte burst from sensor registers.
 * @param raw Pointer to destination raw data structure.
 * @return true on success.
 */
bool MPU6050_ReadRaw(imu_raw_data_t *raw);

/**
 * @brief Read calibrated physical metrics (accel in m/s^2, gyro in rad/s, temp in °C, orientation quaternion).
 * @param data Pointer to output imu_data_t structure.
 * @return true on success.
 */
bool MPU6050_ReadCalibrated(imu_data_t *data);

/**
 * @brief Accumulate sensor samples at rest to calculate zero-rate gyro and accel bias.
 * @param samples Number of samples to average (e.g. 100 to 500).
 * @return true on success.
 */
bool MPU6050_CalibrateBias(uint16_t samples);

/**
 * @brief Get the imu_interface_t function pointer table.
 * @return Pointer to imu_interface_t instance.
 */
const imu_interface_t* MPU6050_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_DRIVER_H */
