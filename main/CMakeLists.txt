idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES esp_driver_gpio esp_adc esp_timer mpu6050 estimator stabiliser wifi_control)
