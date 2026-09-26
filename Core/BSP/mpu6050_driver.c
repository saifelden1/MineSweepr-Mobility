/**
 * @file mpu6050_driver.c
 * @brief Concrete BSP Driver implementation for MPU-6050 6-DOF IMU.
 */

#include "mpu6050_driver.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

static float s_accel_bias_x = 0.0f;
static float s_accel_bias_y = 0.0f;
static float s_accel_bias_z = 0.0f;
static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;
static bool s_is_calibrated = false;
static bool s_is_initialized = false;

/* Orientation quaternion state */
static float s_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // [w, x, y, z]
static uint32_t s_last_timestamp_ms = 0;

static HAL_StatusTypeDef i2c_write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0;
    if (i2c_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1) != HAL_OK)
    {
        return false;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL && who_am_i != MPU6500_WHO_AM_I_VAL)
    {
        return false;
    }

    /* 1. Wake up device, select internal PLL with X-axis gyroscope reference */
    if (i2c_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK)
    {
        return false;
    }
    HAL_Delay(10);

    /* 2. Configure DLPF Mode 3: Accel BW 44 Hz, Gyro BW 42 Hz */
    if (i2c_write_reg(MPU6050_REG_CONFIG, 0x03) != HAL_OK)
    {
        return false;
    }

    /* 3. Sample Rate Divider = 19 (1000 / (1 + 19) = 50 Hz output rate) */
    if (i2c_write_reg(MPU6050_REG_SMPLRT_DIV, 0x13) != HAL_OK)
    {
        return false;
    }

    /* 4. Gyro Config: Full scale +-2000 deg/s (FS_SEL = 3 -> 0x18) */
    if (i2c_write_reg(MPU6050_REG_GYRO_CONFIG, 0x18) != HAL_OK)
    {
        return false;
    }

    /* 5. Accel Config: Full scale +-2g (AFS_SEL = 0 -> 0x00) */
    if (i2c_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK)
    {
        return false;
    }

    s_is_initialized = true;
    s_last_timestamp_ms = HAL_GetTick();
    return true;
}

bool MPU6050_ReadRaw(imu_raw_data_t *raw)
{
    if (!s_is_initialized || raw == NULL)
    {
        return false;
    }

    uint8_t buffer[14];
    if (i2c_read_regs(MPU6050_REG_ACCEL_XOUT_H, buffer, 14) != HAL_OK)
    {
        return false;
    }

    raw->accel_x_raw = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    raw->accel_y_raw = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    raw->accel_z_raw = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    raw->temp_raw    = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);
    raw->gyro_x_raw  = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    raw->gyro_y_raw  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    raw->gyro_z_raw  = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    return true;
}

bool MPU6050_ReadCalibrated(imu_data_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    imu_raw_data_t raw;
    if (!MPU6050_ReadRaw(&raw))
    {
        return false;
    }

    uint32_t now_ms = HAL_GetTick();
    float dt = (float)(now_ms - s_last_timestamp_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f)
    {
        dt = 0.02f; /* fallback to nominal 50 Hz dt */
    }
    s_last_timestamp_ms = now_ms;

    /* Physical metric conversions */
    float ax = ((float)raw.accel_x_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS - s_accel_bias_x;
    float ay = ((float)raw.accel_y_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS - s_accel_bias_y;
    float az = ((float)raw.accel_z_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS - s_accel_bias_z;

    float gx = (((float)raw.gyro_x_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD) - s_gyro_bias_x;
    float gy = (((float)raw.gyro_y_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD) - s_gyro_bias_y;
    float gz = (((float)raw.gyro_z_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD) - s_gyro_bias_z;

    float temp = ((float)raw.temp_raw / 340.0f) + 36.53f;

    /* Simple quaternion integration from angular rates */
    float half_dt = 0.5f * dt;
    float q0 = s_quat[0];
    float q1 = s_quat[1];
    float q2 = s_quat[2];
    float q3 = s_quat[3];

    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    /* Normalize quaternion */
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.00001f)
    {
        s_quat[0] = q0 / norm;
        s_quat[1] = q1 / norm;
        s_quat[2] = q2 / norm;
        s_quat[3] = q3 / norm;
    }

    data->accel_x_m_s2 = ax;
    data->accel_y_m_s2 = ay;
    data->accel_z_m_s2 = az;
    data->gyro_x_rad_s = gx;
    data->gyro_y_rad_s = gy;
    data->gyro_z_rad_s = gz;
    data->temp_deg_c   = temp;
    data->orientation_quat[0] = s_quat[0];
    data->orientation_quat[1] = s_quat[1];
    data->orientation_quat[2] = s_quat[2];
    data->orientation_quat[3] = s_quat[3];
    data->timestamp_ms = now_ms;

    return true;
}

bool MPU6050_CalibrateBias(uint16_t samples)
{
    if (samples == 0)
    {
        samples = 100;
    }

    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        imu_raw_data_t raw;
        if (MPU6050_ReadRaw(&raw))
        {
            sum_ax += ((float)raw.accel_x_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS;
            sum_ay += ((float)raw.accel_y_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS;
            sum_az += ((float)raw.accel_z_raw / MPU6050_ACCEL_SENS_2G) * GRAVITY_MSS;

            sum_gx += ((float)raw.gyro_x_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD;
            sum_gy += ((float)raw.gyro_y_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD;
            sum_gz += ((float)raw.gyro_z_raw / MPU6050_GYRO_SENS_2000DPS) * DEG_TO_RAD;

            valid_samples++;
        }
        HAL_Delay(5);
    }

    if (valid_samples == 0)
    {
        return false;
    }

    s_accel_bias_x = sum_ax / (float)valid_samples;
    s_accel_bias_y = sum_ay / (float)valid_samples;
    /* On level ground, Z reads +1g, so bias is difference from GRAVITY_MSS */
    s_accel_bias_z = (sum_az / (float)valid_samples) - GRAVITY_MSS;

    s_gyro_bias_x = sum_gx / (float)valid_samples;
    s_gyro_bias_y = sum_gy / (float)valid_samples;
    s_gyro_bias_z = sum_gz / (float)valid_samples;

    s_is_calibrated = true;
    return true;
}

static const imu_interface_t s_mpu6050_interface = {
    .init            = MPU6050_Init,
    .read_calibrated = MPU6050_ReadCalibrated,
    .calibrate_bias  = MPU6050_CalibrateBias,
};

const imu_interface_t* MPU6050_GetInterface(void)
{
    return &s_mpu6050_interface;
}
