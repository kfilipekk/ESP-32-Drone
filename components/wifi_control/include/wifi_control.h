#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise Wi-Fi Access Point and Web Server
 */
void wifi_control_init(void);

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    
    int has_tuning;
    int tuning_id;
    float kp;
    float ki;
    float kd;

} wifi_control_data_t;

/**
 * @brief Get the latest control values
 * 
 * @param control Pointer to structure to fill
 * @return int 1 if new data is available, 0 otherwise
 */
int wifi_control_get_data(wifi_control_data_t *control);

void wifi_control_send_telemetry(float roll, float pitch, float yaw, float voltage, 
                                 int16_t ax, int16_t ay, int16_t az, 
                                 int16_t gx, int16_t gy, int16_t gz,
                                 float m1, float m2, float m3, float m4,
                                 float p_term, float i_term, float d_term);

#ifdef __cplusplus
}
#endif
