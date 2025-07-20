#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise Wi-Fi Access Point and Web Server
 */
void wifi_control_init(void);

/**
 * @brief Get the latest control values
 * 
 * @param throttle Pointer to float (0.0 to 100.0)
 * @param steering Pointer to float (-100.0 to 100.0)
 * @return int 1 if new data is available, 0 otherwise
 */
int wifi_control_get_data(float *throttle, float *steering);

#ifdef __cplusplus
}
#endif
