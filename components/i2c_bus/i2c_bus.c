#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "I2C_BUS";

static void i2c_bus_recover(void) {
    //manual recovery for stuck sda
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
    }
    //stop condition
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
}

esp_err_t i2c_bus_init(void)
{
    i2c_bus_recover();

    int i2c_master_port = I2C_MASTER_NUM;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0, //clear clock flags
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return err;
    }

    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return err;
    }

    ESP_LOGI(TAG, "I2C bus initialised SCL:%d SDA:%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    return ESP_OK;
}

esp_err_t i2c_bus_write(uint8_t device_address, uint8_t *write_buffer, size_t write_len)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, device_address, write_buffer, write_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_bus_read(uint8_t device_address, uint8_t *read_buffer, size_t read_len)
{
    return i2c_master_read_from_device(I2C_MASTER_NUM, device_address, read_buffer, read_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_bus_write_byte(uint8_t device_address, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_bus_write(device_address, write_buf, sizeof(write_buf));
}

esp_err_t i2c_bus_read_bytes(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, device_address, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
