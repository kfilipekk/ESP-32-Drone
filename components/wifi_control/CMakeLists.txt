idf_component_register(SRCS "wifi_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash esp_wifi esp_http_server json)
