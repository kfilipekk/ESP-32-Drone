idf_component_register(SRCS "wifi_control.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_event espressif__esp_websocket_client
                       PRIV_REQUIRES esp_wifi esp_netif nvs_flash mbedtls espressif__cjson)
