/**
 * @file mpu6050_driver.c
 * @brief MPU-6050 6-DOF IMU Driver Implementation over I2C2.
 */

#include "mpu6050_driver.h"
#include <string.h>

#define MPU6050_REG_CONFIG       (0x1AU)
#define MPU6050_REG_GYRO_CONFIG  (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H (0x3BU)
#define MPU6050_REG_PWR_MGMT_1   (0x6BU)

#define GYRO_SCALE_FACTOR        (131.0f)   /**< LSB per deg/s for +/-250 deg/s range */
#define ACCEL_SCALE_FACTOR       (16384.0f) /**< LSB per g for +/-2g range */
#define DEG_TO_RAD               (0.017453292519943295f)
#define GRAVITY_M_S2             (9.80665f)

static I2C_HandleTypeDef *s_hi2c = NULL;

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    if (s_hi2c == NULL) return false;

    /* Wake up MPU-6050 (clear SLEEP bit in PWR_MGMT_1) */
    uint8_t pwr_mgmt = 0x00;
    if (HAL_I2C_Mem_Write(s_hi2c, IMU_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_1, 1, &pwr_mgmt, 1, 100) != HAL_OK)
    {
        return false;
    }

    /* Configure DLPF (Digital Low Pass Filter) to 42 Hz (register 0x1A = 0x03) */
    uint8_t dlpf_cfg = 0x03;
    if (HAL_I2C_Mem_Write(s_hi2c, IMU_I2C_ADDRESS, MPU6050_REG_CONFIG, 1, &dlpf_cfg, 1, 100) != HAL_OK)
    {
        return false;
    }

    /* Configure Gyroscope for +/-250 dps (register 0x1B = 0x00) */
    uint8_t gyro_cfg = 0x00;
    if (HAL_I2C_Mem_Write(s_hi2c, IMU_I2C_ADDRESS, MPU6050_REG_GYRO_CONFIG, 1, &gyro_cfg, 1, 100) != HAL_OK)
    {
        return false;
    }

    /* Configure Accelerometer for +/-2g (register 0x1C = 0x00) */
    uint8_t accel_cfg = 0x00;
    if (HAL_I2C_Mem_Write(s_hi2c, IMU_I2C_ADDRESS, MPU6050_REG_ACCEL_CONFIG, 1, &accel_cfg, 1, 100) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool MPU6050_ReadData(mpu6050_data_t *data)
{
    if (s_hi2c == NULL || data == NULL) return false;

    uint8_t raw[14];
    if (HAL_I2C_Mem_Read(s_hi2c, IMU_I2C_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, 1, raw, 14, 50) != HAL_OK)
    {
        return false;
    }

    int16_t ax_raw = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    int16_t gx_raw = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

    data->accel_x_m_s2 = ((float)ax_raw / ACCEL_SCALE_FACTOR) * GRAVITY_M_S2;
    data->accel_y_m_s2 = ((float)ay_raw / ACCEL_SCALE_FACTOR) * GRAVITY_M_S2;
    data->accel_z_m_s2 = ((float)az_raw / ACCEL_SCALE_FACTOR) * GRAVITY_M_S2;

    data->gyro_x_rad_s = ((float)gx_raw / GYRO_SCALE_FACTOR) * DEG_TO_RAD;
    data->gyro_y_rad_s = ((float)gy_raw / GYRO_SCALE_FACTOR) * DEG_TO_RAD;
    data->gyro_z_rad_s = ((float)gz_raw / GYRO_SCALE_FACTOR) * DEG_TO_RAD;

    /* Unit quaternion for planar mobile robot */
    data->orientation_quat[0] = 1.0f; // w
    data->orientation_quat[1] = 0.0f; // x
    data->orientation_quat[2] = 0.0f; // y
    data->orientation_quat[3] = 0.0f; // z

    return true;
}
