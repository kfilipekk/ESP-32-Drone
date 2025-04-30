#include "motor.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

//LEDC Configuration
#define MOTOR_LEDC_TIMER        LEDC_TIMER_0
#define MOTOR_LEDC_MODE         LEDC_LOW_SPEED_MODE 
#define MOTOR_LEDC_DUTY_RES     LEDC_TIMER_12_BIT
#define MOTOR_LEDC_FREQUENCY    15000

//structure to map motor ID to channel
typedef struct {
    int pin;
    ledc_channel_t channel;
} motor_config_t;

static const motor_config_t motors[4] = {
    { .pin = MOTOR_M1_GPIO, .channel = LEDC_CHANNEL_0 },
    { .pin = MOTOR_M2_GPIO, .channel = LEDC_CHANNEL_1 },
    { .pin = MOTOR_M3_GPIO, .channel = LEDC_CHANNEL_2 },
    { .pin = MOTOR_M4_GPIO, .channel = LEDC_CHANNEL_3 }
};

void motor_init(void)
{
    //1.Timer Config
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = MOTOR_LEDC_MODE,
        .duty_resolution  = MOTOR_LEDC_DUTY_RES,
        .timer_num        = MOTOR_LEDC_TIMER,
        .freq_hz          = MOTOR_LEDC_FREQUENCY, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    //2.Channel Config
    for (int i=0; i<4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = MOTOR_LEDC_MODE,
            .channel        = motors[i].channel,
            .timer_sel      = MOTOR_LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motors[i].pin,
            .duty           = 0, // 0%
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
    
    ESP_LOGI(TAG, "Motors initialised on pins %d, %d, %d, %d", 
             MOTOR_M1_GPIO, MOTOR_M2_GPIO, MOTOR_M3_GPIO, MOTOR_M4_GPIO);
}

void motor_set_thrust(int motor_id, float duty_percent)
{
    if(motor_id < 0 || motor_id > 3) return;

    if(duty_percent < 0) duty_percent = 0;
    if(duty_percent > 100) duty_percent = 100;

    //map 0-100 to 0-4095 (12-bit)
    uint32_t duty = (uint32_t)(duty_percent * 4095.0f / 100.0f);
    
    ledc_set_duty(MOTOR_LEDC_MODE, motors[motor_id].channel, duty);
    ledc_update_duty(MOTOR_LEDC_MODE, motors[motor_id].channel);
}

void motor_set_all(float m1, float m2, float m3, float m4)
{
    motor_set_thrust(0, m1);
    motor_set_thrust(1, m2);
    motor_set_thrust(2, m3);
    motor_set_thrust(3, m4);
}

void motor_stop_all(void)
{
    motor_set_all(0, 0, 0, 0);
}
