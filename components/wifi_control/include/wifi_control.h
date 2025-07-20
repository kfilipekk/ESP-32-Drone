#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//Initialise Wi-Fi Access Point and Web Server
void wifi_control_init(void);

//get the latest control values
int wifi_control_get_data(float *throttle, float *steering);

#ifdef __cplusplus
}
#endif
