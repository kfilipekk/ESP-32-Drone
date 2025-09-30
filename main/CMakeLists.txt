idf_component_register(SRCS "main.cpp" "drone_system.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd ssd1306 i2c_bus mpu6050 motor wifi_control pid)

