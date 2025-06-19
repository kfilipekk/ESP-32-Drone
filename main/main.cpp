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

    ESP_LOGI(TAG, "Logs active. Press UP/DOWN for throttle, LEFT/RIGHT for steering, SPACE to stop.");

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    float throttle = 0.0f;
    float steering = 0.0f;

    while (1) {
        int c = getchar();
        
        if (c != EOF) {
            bool update = false;

            if (c == ' ') {
                throttle = 0.0f;
                steering = 0.0f;
                update = true;
                ESP_LOGI(TAG, "STOP");
            }
            else if (c == 0x1B) {
                int c2 = getchar();
                if (c2 == '[') {
                    int c3 = getchar();
                    switch(c3) {
                        case 'A':
                            throttle += 5.0f;
                            if(throttle > 100.0f) throttle = 100.0f;
                            update = true;
                            ESP_LOGI(TAG, "Throttle UP: %.1f", throttle);
                            break;
                        case 'B':
                            throttle -= 5.0f;
                            if(throttle < 0.0f) throttle = 0.0f;
                            update = true;
                            ESP_LOGI(TAG, "Throttle DOWN: %.1f", throttle);
                            break;
                        case 'C':
                            steering += 5.0f;
                            update = true;
                            ESP_LOGI(TAG, "Right: %.1f", steering);
                            break;
                        case 'D':
                            steering -= 5.0f;
                            update = true;
                            ESP_LOGI(TAG, "Left: %.1f", steering);
                            break;
                    }
                }
            }

            if (update) {
                float m1 = throttle + steering;
                float m2 = throttle - steering;
                float m3 = throttle - steering;
                float m4 = throttle + steering;
                
                motor_set_all(m1, m2, m3, m4);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); //prevent watchdog starvation
    }
}