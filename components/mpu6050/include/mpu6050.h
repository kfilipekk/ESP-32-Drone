#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include <span>
#include "driver/i2c_master.h"
#include "esp_err.h"

namespace drone {

struct ImuSample {
    std::int16_t accel_x;
    std::int16_t accel_y;
    std::int16_t accel_z;
    std::int16_t gyro_x;
    std::int16_t gyro_y;
    std::int16_t gyro_z;
};

class Mpu6050 {
public:
    esp_err_t init();
    esp_err_t read(ImuSample& sample) const;
    void set_gyro_offsets(std::int16_t x_offset, std::int16_t y_offset, std::int16_t z_offset);

private:
    esp_err_t write_register(std::uint8_t reg, std::uint8_t value) const;
    esp_err_t read_registers(std::uint8_t reg, std::span<std::uint8_t> data) const;

    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t device_{nullptr};
    std::int16_t gx_offset_{0};
    std::int16_t gy_offset_{0};
    std::int16_t gz_offset_{0};
};

}

#endif
