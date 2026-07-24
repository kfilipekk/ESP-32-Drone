#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <mutex>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "estimator.h"
#include "mpu6050.h"
#include "stabiliser.h"
#include "wifi_control.h"

namespace drone {
namespace {

constexpr const char* tag{"DRONE_SYSTEM"};

constexpr gpio_num_t led_red{GPIO_NUM_8};
constexpr gpio_num_t led_green{GPIO_NUM_9};
constexpr gpio_num_t led_blue{GPIO_NUM_7};

constexpr adc_channel_t battery_channel{ADC_CHANNEL_0};
constexpr float adc_full_scale{4095.0f};
constexpr float adc_reference_volts{3.3f};
constexpr float battery_divider{6.0f};

constexpr TickType_t flight_period{(pdMS_TO_TICKS(1U) > 0U) ? pdMS_TO_TICKS(1U) : static_cast<TickType_t>(1U)};
constexpr std::uint32_t flight_stack_bytes{4096U};
constexpr UBaseType_t flight_priority{5U};
constexpr BaseType_t flight_core{0};

constexpr float us_per_s{1000000.0f};
constexpr float fallback_dt{0.001f};
constexpr float max_dt{0.1f};
constexpr float tilt_limit_deg{45.0f};
constexpr float rearm_throttle{2.0f};
constexpr std::uint32_t telemetry_divider{50U};

constexpr std::uint32_t warmup_samples{50U};
constexpr std::uint32_t calibration_samples{200U};
constexpr std::uint32_t blink_interval{10U};

void configure_led(const gpio_num_t pin)
{
    static_cast<void>(gpio_reset_pin(pin));
    static_cast<void>(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
}

void set_led(const gpio_num_t pin, const bool on)
{
    static_cast<void>(gpio_set_level(pin, on ? 1U : 0U));
}

std::int16_t average(const std::int32_t sum, const std::int32_t count)
{
    return static_cast<std::int16_t>(sum / count);
}

class DroneSystem {
public:
    [[noreturn]] void run();

private:
    [[noreturn]] static void flight_task(void* arg);
    [[noreturn]] void flight_loop();
    void fly(const ImuSample& imu, float dt);
    void send_telemetry(const ImuSample& imu);
    void calibrate_gyro();
    void init_battery_adc();
    float battery_voltage() const;
    void publish(const RemoteCommand& remote);

    Mpu6050 imu_;
    Estimator estimator_;
    Stabiliser stabiliser_;
    WifiControl wifi_;
    adc_oneshot_unit_handle_t adc_{nullptr};
    bool emergency_stop_{false};

    //shared between the main loop and the flight task
    std::mutex mutex_;
    ControlCommand command_{};
    RemoteCommand tuning_{};
};

void DroneSystem::run()
{
    ESP_LOGI(tag, "Init");

    configure_led(led_red);
    configure_led(led_green);
    configure_led(led_blue);
    set_led(led_blue, true);

    wifi_.init();
    motor_init();
    init_battery_adc();

    if (imu_.init() == ESP_OK) {
        ESP_LOGI(tag, "IMU active!");
        calibrate_gyro();
    } else {
        ESP_LOGE(tag, "IMU failed!");
        set_led(led_blue, false);
        set_led(led_red, true);
    }

    if (xTaskCreatePinnedToCore(&DroneSystem::flight_task, "flight_task", flight_stack_bytes, this, flight_priority,
                                nullptr, flight_core) != pdPASS) {
        ESP_LOGE(tag, "Flight task creation failed");
    }

    for (;;) {
        RemoteCommand remote{};
        if (wifi_.poll(remote)) {
            ESP_LOGI(tag, "WiFi -> T:%.1f, R:%.1f, P:%.1f, Y:%.1f", static_cast<double>(remote.throttle),
                     static_cast<double>(remote.roll), static_cast<double>(remote.pitch),
                     static_cast<double>(remote.yaw));
            if (remote.has_tuning) {
                ESP_LOGW(tag, "Tuning ID:%" PRId32 " P:%.2f I:%.2f D:%.2f", remote.tuning_id,
                         static_cast<double>(remote.kp), static_cast<double>(remote.ki),
                         static_cast<double>(remote.kd));
            }
            publish(remote);
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

void DroneSystem::publish(const RemoteCommand& remote)
{
    const std::lock_guard<std::mutex> lock{mutex_};
    command_.throttle = remote.throttle;
    command_.roll = remote.roll;
    command_.pitch = remote.pitch;
    command_.yaw = remote.yaw;
    if (remote.has_tuning) {
        tuning_ = remote;
    }
}

void DroneSystem::flight_task(void* const arg)
{
    static_cast<DroneSystem*>(arg)->flight_loop();
}

void DroneSystem::flight_loop()
{
    TickType_t last_wake{xTaskGetTickCount()};
    std::int64_t last_time{esp_timer_get_time()};
    std::uint32_t telemetry_count{0U};
    ImuSample imu{};

    for (;;) {
        const std::int64_t now{esp_timer_get_time()};
        const float elapsed{static_cast<float>(now - last_time) / us_per_s};
        last_time = now;

        if (imu_.read(imu) == ESP_OK) {
            fly(imu, (elapsed > 0.0f) ? std::fmin(elapsed, max_dt) : fallback_dt);

            ++telemetry_count;
            if (telemetry_count >= telemetry_divider) {
                telemetry_count = 0U;
                send_telemetry(imu);
            }
        }
        static_cast<void>(xTaskDelayUntil(&last_wake, flight_period));
    }
}

void DroneSystem::fly(const ImuSample& imu, const float dt)
{
    estimator_.update(imu, dt);
    const Attitude& attitude{estimator_.attitude()};

    ControlCommand command{};
    RemoteCommand tuning{};
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        command = command_;
        tuning = tuning_;
        tuning_.has_tuning = false;
    }
    if (tuning.has_tuning) {
        stabiliser_.tune(tuning.tuning_id, tuning.kp, tuning.ki, tuning.kd);
    }

    const bool tilted{(std::fabs(attitude.roll) > tilt_limit_deg) || (std::fabs(attitude.pitch) > tilt_limit_deg)};
    if (!emergency_stop_ && tilted) {
        ESP_LOGE(tag, "EMERGENCY: Tilt > 45 deg. KILLED.");
        emergency_stop_ = true;
    } else if (emergency_stop_ && !tilted && (command.throttle < rearm_throttle)) {
        ESP_LOGI(tag, "Emergency Stop Reset.");
        emergency_stop_ = false;
    } else {
        //no change
    }

    //zero throttle keeps motors off and resets the pids until the pilot throttles up again
    if (emergency_stop_) {
        command.throttle = 0.0f;
        const std::lock_guard<std::mutex> lock{mutex_};
        command_.throttle = 0.0f;
    }

    stabiliser_.run(attitude, command);
}

void DroneSystem::send_telemetry(const ImuSample& imu)
{
    const Attitude& attitude{estimator_.attitude()};
    const Pid& pid{stabiliser_.rate_pitch()};

    Telemetry telemetry{};
    telemetry.roll = attitude.roll;
    telemetry.pitch = attitude.pitch;
    telemetry.yaw = attitude.yaw;
    telemetry.voltage = battery_voltage();
    telemetry.ax = imu.accel_x;
    telemetry.ay = imu.accel_y;
    telemetry.az = imu.accel_z;
    telemetry.gx = imu.gyro_x;
    telemetry.gy = imu.gyro_y;
    telemetry.gz = imu.gyro_z;
    telemetry.motors = stabiliser_.motor_outputs();
    telemetry.p_term = pid.last_p();
    telemetry.i_term = pid.integral();
    telemetry.d_term = pid.last_d();

    wifi_.send(telemetry);
}

void DroneSystem::calibrate_gyro()
{
    ESP_LOGI(tag, "Waiting 2s before calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000U));
    ESP_LOGI(tag, "Calibrating Gyro... Keep drone still!");

    ImuSample sample{};
    for (std::uint32_t i{0U}; i < warmup_samples; ++i) {
        static_cast<void>(imu_.read(sample));
        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    std::int32_t sum_x{0};
    std::int32_t sum_y{0};
    std::int32_t sum_z{0};
    std::int32_t count{0};
    bool led_on{false};

    for (std::uint32_t i{0U}; i < calibration_samples; ++i) {
        if (imu_.read(sample) == ESP_OK) {
            sum_x += sample.gyro_x;
            sum_y += sample.gyro_y;
            sum_z += sample.gyro_z;
            ++count;
        }

        //blink red while sampling
        if ((i % blink_interval) == 0U) {
            led_on = !led_on;
            set_led(led_red, led_on);
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    if (count > 0) {
        imu_.set_gyro_offsets(average(sum_x, count), average(sum_y, count), average(sum_z, count));
    } else {
        ESP_LOGE(tag, "Calibration Failed - No valid data");
    }

    set_led(led_red, false);
    set_led(led_green, true);
}

void DroneSystem::init_battery_adc()
{
    adc_oneshot_unit_init_cfg_t unit_config{};
    unit_config.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_));

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_, battery_channel, &channel_config));
}

float DroneSystem::battery_voltage() const
{
    int raw{0};
    if (adc_oneshot_read(adc_, battery_channel, &raw) != ESP_OK) {
        raw = 0;
    }
    return (static_cast<float>(raw) / adc_full_scale) * adc_reference_volts * battery_divider;
}

}
}

extern "C" void app_main()
{
    drone::DroneSystem drone_system{};
    drone_system.run();
}
