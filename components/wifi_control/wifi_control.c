#include <string.h>
#include "wifi_control.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"

static const char *TAG = "WIFI_CONTROL";

static float s_throttle = 0.0f;
static float s_steering = 0.0f;
static int s_new_data = 0;

static esp_websocket_client_handle_t client = NULL;

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA Op: 0x%x Len: %d", data->op_code, data->data_len);
        if (data->op_code == 0x01 && data->data_len > 0) {
            ESP_LOGI(TAG, "Payload: %.*s", data->data_len, data->data_ptr);
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (root) {
                cJSON *t = cJSON_GetObjectItem(root, "t");
                cJSON *s = cJSON_GetObjectItem(root, "s");
                if (t && s) {
                    s_throttle = (float)t->valuedouble;
                    s_steering = (float)s->valuedouble;
                    s_new_data = 1;
                } else {
                    ESP_LOGE(TAG, "JSON missing keys");
                }
                cJSON_Delete(root);
            } else {
                ESP_LOGE(TAG, "JSON parse failed");
            }
        }
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
        break;
    }
}

static void start_websocket_client(void)
{
    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = "wss://krystianfilipek.com/ws?role=drone";
    websocket_cfg.transport = WEBSOCKET_TRANSPORT_OVER_SSL;
    websocket_cfg.crt_bundle_attach = esp_crt_bundle_attach;

    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);
    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);
}

/* --- Wi-Fi Event Handler --- */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Retrying");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Drone Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        //start WebSocket client only after we have an IP
        start_websocket_client();
    }
}

/* --- Wi-Fi Init (Station) --- */
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //disable power save for now- better response time and to prevent "unreachable" state
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void wifi_control_init(void)
{
    //NVS is required for Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
}

int wifi_control_get_data(float *throttle, float *steering)
{
    *throttle = s_throttle;
    *steering = s_steering;
    int ret = s_new_data;
    s_new_data = 0;
    return ret;
}
