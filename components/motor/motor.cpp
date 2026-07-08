#include "motor.h"
#include <algorithm>
#include <cstdint>
#include "driver/ledc.h"
#include "esp_log.h"

namespace drone {
namespace {

constexpr const char* tag{"MOTOR"};

constexpr ledc_timer_t timer{LEDC_TIMER_0};
constexpr ledc_mode_t speed_mode{LEDC_LOW_SPEED_MODE};
constexpr std::uint32_t frequency_hz{15000U};
constexpr float max_duty{4095.0f};
constexpr float max_percent{100.0f};

struct MotorPin {
    int pin;
    ledc_channel_t channel;
};

constexpr std::array<MotorPin, motor_count> motors{{
    {4, LEDC_CHANNEL_0},
    {5, LEDC_CHANNEL_1},
    {6, LEDC_CHANNEL_2},
    {3, LEDC_CHANNEL_3},
}};

}

void motor_init()
{
    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = speed_mode;
    timer_config.duty_resolution = LEDC_TIMER_12_BIT;
    timer_config.timer_num = timer;
    timer_config.freq_hz = frequency_hz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (const MotorPin& motor : motors) {
        ledc_channel_config_t channel_config{};
        channel_config.speed_mode = speed_mode;
        channel_config.channel = motor.channel;
        channel_config.timer_sel = timer;
        channel_config.gpio_num = motor.pin;
        channel_config.duty = 0U;
        channel_config.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    ESP_LOGI(tag, "Motors initialised on pins %d, %d, %d, %d", motors[0U].pin, motors[1U].pin, motors[2U].pin,
             motors[3U].pin);
}

void motor_set_all(const MotorDuty& duty)
{
    for (std::size_t i{0U}; i < motor_count; ++i) {
        const float percent{std::clamp(duty[i], 0.0f, max_percent)};
        const auto counts{static_cast<std::uint32_t>((percent * max_duty) / max_percent)};
        static_cast<void>(ledc_set_duty(speed_mode, motors[i].channel, counts));
        static_cast<void>(ledc_update_duty(speed_mode, motors[i].channel));
    }
}

}
