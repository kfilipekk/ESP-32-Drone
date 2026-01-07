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
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;

static int s_has_tuning = 0;
static int s_tuning_id = 0;
static float s_kp = 0.0f;
static float s_ki = 0.0f;
static float s_kd = 0.0f;

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
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (root) {
                cJSON *t = cJSON_GetObjectItem(root, "t");
                if (!t) t = cJSON_GetObjectItem(root, "throttle");
                
                cJSON *r = cJSON_GetObjectItem(root, "r");
                if (!r) r = cJSON_GetObjectItem(root, "roll");
                
                cJSON *p = cJSON_GetObjectItem(root, "p");
                if (!p) p = cJSON_GetObjectItem(root, "pitch");
                
                cJSON *y = cJSON_GetObjectItem(root, "y");
                if (!y) y = cJSON_GetObjectItem(root, "yaw");
                
                if (t) s_throttle = (float)t->valuedouble;
                if (r) s_roll = (float)r->valuedouble;
                if (p) s_pitch = (float)p->valuedouble;
                if (y) s_yaw = (float)y->valuedouble;

                //tuning packets: { "tid": 0, "kp": 1.0, "ki": 0.0, "kd": 0.0 }
                cJSON *tid = cJSON_GetObjectItem(root, "tid");
                cJSON *kp_json = cJSON_GetObjectItem(root, "kp");
                cJSON *ki_json = cJSON_GetObjectItem(root, "ki");
                cJSON *kd_json = cJSON_GetObjectItem(root, "kd");

                if (tid && kp_json && ki_json && kd_json) {
                    s_has_tuning = 1;
                    s_tuning_id = tid->valueint;
                    s_kp = (float)kp_json->valuedouble;
                    s_ki = (float)ki_json->valuedouble;
                    s_kd = (float)kd_json->valuedouble;
                }
                
                s_new_data = 1;
                cJSON_Delete(root);
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
    
    //Critical for low latency over internet
    websocket_cfg.network_timeout_ms = 5000;
    
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

int wifi_control_get_data(wifi_control_data_t *control)
{
    control->throttle = s_throttle;
    control->roll = s_roll;
    control->pitch = s_pitch;
    control->yaw = s_yaw;
    
    //pass tuning data
    control->has_tuning = s_has_tuning;
    if (s_has_tuning) {
        control->tuning_id = s_tuning_id;
        control->kp = s_kp;
        control->ki = s_ki;
        control->kd = s_kd;
        s_has_tuning = 0; //consume event
    }

    int ret = s_new_data;
    s_new_data = 0;
    return ret;
}

void wifi_control_send_telemetry(float roll, float pitch, float yaw, float voltage, 
                                 int16_t ax, int16_t ay, int16_t az, 
                                 int16_t gx, int16_t gy, int16_t gz,
                                 float m1, float m2, float m3, float m4,
                                 float p_term, float i_term, float d_term) {
    if (client == NULL || !esp_websocket_client_is_connected(client)) return;
    char json_buffer[350];
    //minify json for packet size
    //t=1:telem, v:voltage, r/p/y:attitude, pi/ii/di:pid debug
    snprintf(json_buffer, sizeof(json_buffer), 
        "{\"t\":1,\"r\":%.1f,\"p\":%.1f,\"y\":%.1f,\"v\":%.1f,\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"m1\":%.0f,\"m2\":%.0f,\"m3\":%.0f,\"m4\":%.0f,\"pi\":%.1f,\"ii\":%.1f,\"di\":%.1f}", 
        roll, pitch, yaw, voltage, ax, ay, az, gx, gy, gz, m1, m2, m3, m4, p_term, i_term, d_term);
    
    //send with minimal timeout
    esp_websocket_client_send_text(client, json_buffer, strlen(json_buffer), 0);
}
