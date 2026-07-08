#include "mpu6050.h"
#include <array>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace drone {
namespace {

constexpr const char* tag{"MPU6050"};

constexpr gpio_num_t scl_pin{GPIO_NUM_10};
constexpr gpio_num_t sda_pin{GPIO_NUM_11};
constexpr std::uint32_t scl_speed_hz{100000U};
constexpr int timeout_ms{1000};

constexpr std::uint16_t device_address{0x68U};
constexpr std::uint8_t who_am_i_value{0x68U};
constexpr std::uint8_t reg_config{0x1AU};
constexpr std::uint8_t reg_gyro_config{0x1BU};
constexpr std::uint8_t reg_accel_config{0x1CU};
constexpr std::uint8_t reg_accel_xout_h{0x3BU};
constexpr std::uint8_t reg_pwr_mgmt_1{0x6BU};
constexpr std::uint8_t reg_who_am_i{0x75U};

constexpr std::uint8_t pwr_reset{0x80U};
constexpr std::uint8_t pwr_wake{0x00U};
constexpr std::uint8_t dlpf_44hz{0x03U};
constexpr std::uint8_t gyro_500dps{0x08U};
constexpr std::uint8_t accel_8g{0x10U};

void set_pin(const gpio_num_t pin, const std::uint32_t level)
{
    static_cast<void>(gpio_set_level(pin, level));
    esp_rom_delay_us(5U);
}

//clock out a stuck sda then issue a stop condition
void recover_bus()
{
    static_cast<void>(gpio_reset_pin(sda_pin));
    static_cast<void>(gpio_reset_pin(scl_pin));
    static_cast<void>(gpio_set_direction(sda_pin, GPIO_MODE_INPUT_OUTPUT_OD));
    static_cast<void>(gpio_set_direction(scl_pin, GPIO_MODE_INPUT_OUTPUT_OD));
    set_pin(sda_pin, 1U);
    set_pin(scl_pin, 1U);

    for (std::uint32_t i{0U}; i < 9U; ++i) {
        set_pin(scl_pin, 0U);
        set_pin(scl_pin, 1U);
    }

    set_pin(sda_pin, 0U);
    set_pin(scl_pin, 1U);
    set_pin(sda_pin, 1U);
}

std::int16_t to_int16(const std::uint8_t high, const std::uint8_t low)
{
    const auto word{static_cast<std::uint16_t>((static_cast<std::uint32_t>(high) << 8U) | static_cast<std::uint32_t>(low))};
    return static_cast<std::int16_t>(word);
}

std::int16_t subtract(const std::int16_t value, const std::int16_t offset)
{
    return static_cast<std::int16_t>(static_cast<std::int32_t>(value) - static_cast<std::int32_t>(offset));
}

}

esp_err_t Mpu6050::init()
{
    recover_bus();

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = sda_pin;
    bus_config.scl_io_num = scl_pin;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7U;
    bus_config.flags.enable_internal_pullup = 1U;

    esp_err_t ret{i2c_new_master_bus(&bus_config, &bus_)};
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = device_address;
    device_config.scl_speed_hz = scl_speed_hz;

    ret = i2c_master_bus_add_device(bus_, &device_config, &device_);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "I2C device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    //wait for power then force reset
    vTaskDelay(pdMS_TO_TICKS(100U));
    static_cast<void>(write_register(reg_pwr_mgmt_1, pwr_reset));
    vTaskDelay(pdMS_TO_TICKS(100U));

    ret = write_register(reg_pwr_mgmt_1, pwr_wake);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Wake up failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50U));
        ret = write_register(reg_pwr_mgmt_1, pwr_wake);
        if (ret != ESP_OK) {
            ESP_LOGE(tag, "Failed to wake up: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    std::array<std::uint8_t, 1U> id{};
    ret = read_registers(reg_who_am_i, id);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to read id: %s", esp_err_to_name(ret));
        return ret;
    }
    if (id[0U] != who_am_i_value) {
        ESP_LOGE(tag, "Incorrect id 0x%02x", static_cast<unsigned int>(id[0U]));
        return ESP_FAIL;
    }

    ret = write_register(reg_config, dlpf_44hz);
    if (ret == ESP_OK) {
        ret = write_register(reg_gyro_config, gyro_500dps);
    }
    if (ret == ESP_OK) {
        ret = write_register(reg_accel_config, accel_8g);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(tag, "MPU6050 initialised");
    }
    return ret;
}

esp_err_t Mpu6050::read(ImuSample& sample) const
{
    //accel xyz, temp, gyro xyz as big endian pairs
    std::array<std::uint8_t, 14U> raw{};
    const esp_err_t ret{read_registers(reg_accel_xout_h, raw)};

    if (ret == ESP_OK) {
        sample.accel_x = to_int16(raw[0U], raw[1U]);
        sample.accel_y = to_int16(raw[2U], raw[3U]);
        sample.accel_z = to_int16(raw[4U], raw[5U]);
        sample.gyro_x = subtract(to_int16(raw[8U], raw[9U]), gx_offset_);
        sample.gyro_y = subtract(to_int16(raw[10U], raw[11U]), gy_offset_);
        sample.gyro_z = subtract(to_int16(raw[12U], raw[13U]), gz_offset_);
    }
    return ret;
}

void Mpu6050::set_gyro_offsets(const std::int16_t x_offset, const std::int16_t y_offset, const std::int16_t z_offset)
{
    gx_offset_ = x_offset;
    gy_offset_ = y_offset;
    gz_offset_ = z_offset;
    ESP_LOGI(tag, "Gyro offsets X:%d Y:%d Z:%d", static_cast<int>(x_offset), static_cast<int>(y_offset),
             static_cast<int>(z_offset));
}

esp_err_t Mpu6050::write_register(const std::uint8_t reg, const std::uint8_t value) const
{
    const std::array<std::uint8_t, 2U> buffer{reg, value};
    return i2c_master_transmit(device_, buffer.data(), buffer.size(), timeout_ms);
}

esp_err_t Mpu6050::read_registers(const std::uint8_t reg, const std::span<std::uint8_t> data) const
{
    return i2c_master_transmit_receive(device_, &reg, 1U, data.data(), data.size(), timeout_ms);
}

}
