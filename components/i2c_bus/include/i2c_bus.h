#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_SCL_IO           GPIO_NUM_10
#define I2C_MASTER_SDA_IO           GPIO_NUM_11
#define I2C_MASTER_NUM              I2C_NUM_0        //I2C port number
#define I2C_MASTER_FREQ_HZ          100000           //I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                //I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                //I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// initialise i2c master interface
esp_err_t i2c_bus_init(void);

// write bytes to device
esp_err_t i2c_bus_write(uint8_t device_address, uint8_t *write_buffer, size_t write_len);

// read bytes from device
esp_err_t i2c_bus_read(uint8_t device_address, uint8_t *read_buffer, size_t read_len);

// write to specific register
esp_err_t i2c_bus_write_byte(uint8_t device_address, uint8_t reg_addr, uint8_t data);

// read from specific register
esp_err_t i2c_bus_read_bytes(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
