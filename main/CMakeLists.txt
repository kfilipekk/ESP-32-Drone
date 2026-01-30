idf_component_register(SRCS "main.cpp" "drone_system.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver i2c_bus mpu6050 motor wifi_control pid stabiliser estimator esp_adc)


