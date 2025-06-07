#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "motor.h"

static const char *TAG = "DRONE_LEDS";

#define LED_RED_GPIO    GPIO_NUM_8
#define LED_GREEN_GPIO  GPIO_NUM_9
#define LED_BLUE_GPIO   GPIO_NUM_7

void configure_led(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Init");

    motor_init();

    //power-on test for motors
    ESP_LOGI(TAG, "Testing Motors Sequence...");
    for(int i=0; i<4; i++) {
        ESP_LOGI(TAG, "Spinning Motor %d", i);
        motor_set_thrust(i, 30.0f); //30% thrust
        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_set_thrust(i, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Motor Test Complete");

    //initialise I2C Bus
    ESP_ERROR_CHECK(i2c_bus_init());

    //initialise MPU6050
    if (mpu6050_init() == ESP_OK) {
        ESP_LOGI(TAG, "IMU active!");
    } else {
        ESP_LOGE(TAG, "IMU failed!");
    }

    configure_led(LED_RED_GPIO);
    configure_led(LED_GREEN_GPIO);
    configure_led(LED_BLUE_GPIO);

    ESP_LOGI(TAG, "blink");

    while (1) {
        ESP_LOGI(TAG, "Red ON");
        gpio_set_level(LED_RED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_RED_GPIO, 0);

        ESP_LOGI(TAG, "Green ON");
        gpio_set_level(LED_GREEN_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_GREEN_GPIO, 0);

        ESP_LOGI(TAG, "Blue ON");
        gpio_set_level(LED_BLUE_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_BLUE_GPIO, 0);
    }
}