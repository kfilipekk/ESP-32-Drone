#include "drone_system.h"
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "motor.h"
#include "wifi_control.h"
#include "stabiliser.h"
#include "estimator.h"

static const char *TAG = "DRONE_SYSTEM";

#define LED_RED_GPIO    GPIO_NUM_8
#define LED_GREEN_GPIO  GPIO_NUM_9
#define LED_BLUE_GPIO   GPIO_NUM_7

static Stabiliser stabiliser;
static Estimator estimator;
static control_command_t current_command = {0};

static void configure_led(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static adc_oneshot_unit_handle_t adc1_handle = NULL;

float get_battery_voltage() {
    static bool adc_init = false;
    if (!adc_init) {
        adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
        adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
        adc_init = true;
    }
    int raw;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw);
    //scale voltage based on user calibration
    return (raw / 4095.0f) * 3.3f * 6.0f;
}

void flight_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    TickType_t period = pdMS_TO_TICKS(1);
    if (period == 0) period = 1;
    
    const TickType_t xFrequency = period;
    xLastWakeTime = xTaskGetTickCount();

    mpu6050_data_t imu_data;
    int64_t last_time = esp_timer_get_time();

    while(1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f; //cap at 100ms

        if (mpu6050_read(&imu_data) == ESP_OK) {
             estimator.update(imu_data, dt);
             
             state_estimate_t state = {
                 .roll = estimator.get_state().roll,
                 .pitch = estimator.get_state().pitch,
                 .yaw = estimator.get_state().yaw,
                 .roll_rate = estimator.get_state().roll_rate,
                 .pitch_rate = estimator.get_state().pitch_rate,
                 .yaw_rate = estimator.get_state().yaw_rate
             };
             
             //emergency kill switch: tilt > 45 degrees
             static bool emergency_stop = false;

             if (!emergency_stop && (fabs(state.roll) > 45.0f || fabs(state.pitch) > 45.0f)) {
                 ESP_LOGE(TAG, "EMERGENCY: Tilt > 45 deg. KILLED.");
                 emergency_stop = true;
             }
             
             //reset emergency stop if throttle is lowered to 0
             if (emergency_stop && current_command.throttle < 2.0f && fabs(state.roll) < 45.0f && fabs(state.pitch) < 45.0f) {
                 ESP_LOGI(TAG, "Emergency Stop Reset.");
                 emergency_stop = false;
             }

             if (emergency_stop) {
                 motor_stop_all();
                 //force command to 0 to ensure pids reset if run is called
                 current_command.throttle = 0.0f;
                 stabiliser.run(state, current_command); //run with 0 throttle to reset pids
             } else {
                 stabiliser.run(state, current_command);
             }

             static int telem_divider = 0;
             if (telem_divider++ >= 50) {
                 telem_divider = 0;
                 //debug info for pitch rate loop
                 Pid* pitch_pid = stabiliser.get_pid_rate_pitch();
                 
                 wifi_control_send_telemetry(state.roll, state.pitch, state.yaw, get_battery_voltage(),
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    stabiliser.get_motor_output(0), stabiliser.get_motor_output(1),
                    stabiliser.get_motor_output(2), stabiliser.get_motor_output(3),
                    pitch_pid->get_last_p(), pitch_pid->get_last_i(), pitch_pid->get_last_d());
             }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void perform_calibration() {
    ESP_LOGI(TAG, "Waiting 2s before calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Calibrating Gyro... Keep drone still!");
    
    const int samples = 200;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int count = 0;
    mpu6050_data_t raw = {0};
    
    //warm up sensor
    for(int i=0; i<50; i++) {
        mpu6050_read(&raw);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    //blink red led quickly during calibration
    bool led_state = false;

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read(&raw) == ESP_OK) {
            //read raw data (offsets are 0 initially)
            sum_gx += raw.gyro_x;
            sum_gy += raw.gyro_y;
            sum_gz += raw.gyro_z;
            count++;
        }
        
        //blink every 10 samples (50ms)
        if (i % 10 == 0) {
            led_state = !led_state;
            gpio_set_level(LED_RED_GPIO, led_state);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (count > 0) {
        //set offsets
        mpu6050_set_gyro_offsets(sum_gx / count, sum_gy / count, sum_gz / count);
    } else {
        ESP_LOGE(TAG, "Calibration Failed - No valid data");
    }
    
    //indicate done (solid green, red off)
    gpio_set_level(LED_RED_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 1);
}

void drone_system_start(void)
{
    ESP_LOGI(TAG, "Init");

    configure_led(LED_RED_GPIO);
    configure_led(LED_GREEN_GPIO);
    configure_led(LED_BLUE_GPIO);
    
    //indicate start (blue)
    gpio_set_level(LED_BLUE_GPIO, 1);

    wifi_control_init();

    motor_init();
    
    ESP_ERROR_CHECK(i2c_bus_init());

    if (mpu6050_init() == ESP_OK) {
        ESP_LOGI(TAG, "IMU active!");
        perform_calibration();
    } else {
        ESP_LOGE(TAG, "IMU failed!");
        //error state (red)
        gpio_set_level(LED_BLUE_GPIO, 0);
        gpio_set_level(LED_RED_GPIO, 1);
    }
    
    stabiliser.init();
    estimator.init();

    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Logs active - UP/DOWN for throttle -LEFT/RIGHT for steering - SPACE to stop");

    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
    setvbuf(stdout, NULL, _IONBF, 0);

    float throttle = 0.0f;
    float steering = 0.0f;

    //state machine for handling multi-byte ANSI sequences (arrow keys)
    enum InputState {
        STATE_IDLE,
        STATE_GOT_ESC,
        STATE_GOT_BRACKET
    };
    static InputState input_state = STATE_IDLE;


    while (1) {
        wifi_control_data_t data;
        if (wifi_control_get_data(&data)) {
             ESP_LOGI(TAG, "WiFi -> T:%.1f, R:%.1f, P:%.1f, Y:%.1f", 
                data.throttle, data.roll, data.pitch, data.yaw);
             
             current_command.throttle = data.throttle;
             current_command.roll = data.roll;
             current_command.pitch = data.pitch;
             current_command.yaw = data.yaw;

             if (data.has_tuning) {
                 ESP_LOGW(TAG, "Tuning ID:%d P:%.2f I:%.2f D:%.2f", 
                    data.tuning_id, data.kp, data.ki, data.kd);
                 if (data.tuning_id == 0) {
                     stabiliser.get_pid_rate_roll()->set_gains(data.kp, data.ki, data.kd);
                     stabiliser.get_pid_rate_pitch()->set_gains(data.kp, data.ki, data.kd);
                 } else if (data.tuning_id == 1) {
                     stabiliser.get_pid_rate_yaw()->set_gains(data.kp, data.ki, data.kd);
                 } else if (data.tuning_id == 2) {
                     stabiliser.get_pid_angle_roll()->set_gains(data.kp, data.ki, data.kd);
                     stabiliser.get_pid_angle_pitch()->set_gains(data.kp, data.ki, data.kd);
                 }
             }
        }

        int c = getchar();
        
        if (c != EOF) {
            switch (input_state) {
                case STATE_IDLE:
                    if (c == 0x1B) {
                        input_state = STATE_GOT_ESC;
                    } 
                    else if (c == ' ') {
                        throttle = 0.0f;
                        steering = 0.0f;
                        current_command.throttle = 0.0f;
                        current_command.roll = 0.0f;
                        ESP_LOGI(TAG, "STOP");
                    }
                    break;

                case STATE_GOT_ESC:
                    if (c == '[') {
                        input_state = STATE_GOT_BRACKET;
                    } else {
                        input_state = STATE_IDLE;
                    }
                    break;

                case STATE_GOT_BRACKET:
                    switch(c) {
                        case 'A': //up arrow
                            throttle += 5.0f;
                            if(throttle > 100.0f) throttle = 100.0f;
                            current_command.throttle = throttle;
                            ESP_LOGI(TAG, "Throttle UP: %.1f", throttle);
                            break;
                        case 'B': //down arrow
                            throttle -= 5.0f;
                            if(throttle < 0.0f) throttle = 0.0f;
                            current_command.throttle = throttle;
                            ESP_LOGI(TAG, "Throttle DOWN: %.1f", throttle);
                            break;
                        case 'C': //right Arrow
                            steering += 5.0f;
                            current_command.roll = steering;
                            ESP_LOGI(TAG, "Right: %.1f", steering);
                            break;
                        case 'D': //left arrow
                            steering -= 5.0f;
                            current_command.roll = steering;
                            ESP_LOGI(TAG, "Left: %.1f", steering);
                            break;
                    }
                    input_state = STATE_IDLE; //reset to wait for next key
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); //prevent watchdog starvation
    }
}
