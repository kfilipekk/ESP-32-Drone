#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDRESS     0x68

// mpu6050 register map
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

// initialise mpu6050 sensor
esp_err_t mpu6050_init(void);

// read all raw data
esp_err_t mpu6050_read(mpu6050_data_t *data);

// check connection
esp_err_t mpu6050_test_connection(void);

// set manual gyro offsets
void mpu6050_set_gyro_offsets(int16_t x_offset, int16_t y_offset, int16_t z_offset);

#ifdef __cplusplus
}
#endif
