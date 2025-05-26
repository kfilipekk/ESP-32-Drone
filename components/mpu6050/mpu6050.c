#include "mpu6050.h"
#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t data;

    //check device ID
    ret = i2c_bus_read_bytes(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, &data, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (data != 0x68) {
        ESP_LOGE(TAG, "MPU6050 Incorrect ID. Expected 0x68, got 0x%02x", data);
        return ESP_FAIL;
    }

    //wake up device (Clear sleep bit)
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;

    //config: DLPF (Digital Low Pass Filter) to ~44Hz
    //register 26 – Configuration. Bit [2:0] = DLPF_CFG
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_CONFIG, 0x03); 
    if (ret != ESP_OK) return ret;

    //gyro Config: 500 deg/s (Fs = 65.5 LSB/(deg/s))
    //register 27 – Gyroscope Configuration. Bit [4:3] = FS_SEL
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, (1 << 3)); 
    if (ret != ESP_OK) return ret;

    //accel Config: +/- 8g (4096 LSB/g)
    //register 28 – Accelerometer Configuration. Bit [4:3] = AFS_SEL
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, (2 << 3)); 
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialised successfully");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data)
{
    uint8_t raw_data[14];
    //burst read 14 bytes starting from ACCEL_XOUT_H
    //accel X (2), Accel Y (2), Accel Z (2), Temp (2), Gyro X (2), Gyro Y (2), Gyro Z (2)
    esp_err_t ret = i2c_bus_read_bytes(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, raw_data, 14);
    
    if (ret == ESP_OK) {
        data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        
        data->temp = (float)((int16_t)((raw_data[6] << 8) | raw_data[7])) / 340.0f + 36.53f;

        data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
        data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
        data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    }

    return ret;
}

esp_err_t mpu6050_test_connection(void)
{
    uint8_t data;
    return i2c_bus_read_bytes(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, &data, 1);
}
