#include "mpu6050.h"
#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

static int16_t s_gx_offset = 0;
static int16_t s_gy_offset = 0;
static int16_t s_gz_offset = 0;

void mpu6050_set_gyro_offsets(int16_t x_offset, int16_t y_offset, int16_t z_offset) {
    s_gx_offset = x_offset;
    s_gy_offset = y_offset;
    s_gz_offset = z_offset;
    ESP_LOGI(TAG, "Gyro Offsets Set: X:%d Y:%d Z:%d", s_gx_offset, s_gy_offset, s_gz_offset);
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t data;

    ESP_LOGI(TAG, "Attempting MPU6050 Init...");

    //wait for power
    vTaskDelay(pdMS_TO_TICKS(100));

    //force reset
    ESP_LOGI(TAG, "Resetting MPU6050...");
    i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    //wake up
    ESP_LOGI(TAG, "Waking up MPU6050...");
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
    //retry if failed
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wake up failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
        if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to wake up: %s", esp_err_to_name(ret));
             return ret;
        }
    }
    
    //check device id
    ESP_LOGI(TAG, "Checking WHO_AM_I...");
    ret = i2c_bus_read_bytes(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (data != 0x68) {
        ESP_LOGE(TAG, "MPU6050 Incorrect ID. Expected 0x68, got 0x%02x", data);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 Config...");
    //dlpf config 44hz
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_CONFIG, 0x03); 
    if (ret != ESP_OK) return ret;

    //gyro config 500dps
    ret = i2c_bus_write_byte(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, (1 << 3)); 
    if (ret != ESP_OK) return ret;

    //accel config 8g
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

        data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]) - s_gx_offset;
        data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) - s_gy_offset;
        data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) - s_gz_offset;
    }

    return ret;
}

esp_err_t mpu6050_test_connection(void)
{
    uint8_t data;
    return i2c_bus_read_bytes(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, &data, 1);
}
