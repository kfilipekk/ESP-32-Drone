#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDRESS     0x68

//MPU6050 Register Map
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float temp;
} mpu6050_data_t;

/**
 * @brief Initialise the MPU6050 sensor
 * Configures DLPF, Gyro/Accel ranges, and wakes up the device.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Read all raw data (Accel + Gyro + Temp)
 * Efficient burst read.
 */
esp_err_t mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Check connection to MPU6050
 */
esp_err_t mpu6050_test_connection(void);

#ifdef __cplusplus
}
#endif
