#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "DRONE_LEDS";

#define LED_RED_GPIO    GPIO_NUM_23
#define LED_GREEN_GPIO  GPIO_NUM_5
#define LED_BLUE_GPIO   GPIO_NUM_18

void configure_led(int pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Init");

    configure_led(LED_RED_GPIO);
    configure_led(LED_GREEN_GPIO);
    configure_led(LED_BLUE_GPIO);

    ESP_LOGI(TAG, "blink");

    while (1) {
        ESP_LOGI(TAG, "Red ON");
        gpio_set_level(LED_RED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_RED_GPIO, 0);

        ESP_LOGI(TAG, "Green ON");
        gpio_set_level(LED_GREEN_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GREEN_GPIO, 0);

        ESP_LOGI(TAG, "Blue ON");
        gpio_set_level(LED_BLUE_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_BLUE_GPIO, 0);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}