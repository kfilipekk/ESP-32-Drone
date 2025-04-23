#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

//default I2C configuration for ESP32-S2 Mini
#define I2C_MASTER_SCL_IO           GPIO_NUM_35
#define I2C_MASTER_SDA_IO           GPIO_NUM_33
#define I2C_MASTER_NUM              I2C_NUM_0        //I2C port number
#define I2C_MASTER_FREQ_HZ          400000           //I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                //I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                //I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * @brief Initialise the I2C master interface
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Write bytes to an I2C device
 * 
 * @param device_address I2C address of the device
 * @param write_buffer Pointer to data to write
 * @param write_len Length of data to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_bus_write(uint8_t device_address, uint8_t *write_buffer, size_t write_len);

/**
 * @brief Read bytes from an I2C device
 * 
 * @param device_address I2C address of the device
 * @param read_buffer Pointer to buffer for read data
 * @param read_len Length of data to read
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_bus_read(uint8_t device_address, uint8_t *read_buffer, size_t read_len);

/**
 * @brief Write to a specific register of an I2C device
 */
esp_err_t i2c_bus_write_byte(uint8_t device_address, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read from a specific register of an I2C device
 */
esp_err_t i2c_bus_read_bytes(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
