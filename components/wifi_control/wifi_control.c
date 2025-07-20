#include <string.h>
#include "wifi_control.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"

static const char *TAG = "WIFI_CONTROL";

static float s_throttle = 0.0f;
static float s_steering = 0.0f;
static int s_new_data = 0;

static const char *html_content = 
"<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=0'>"
"<style>body{text-align:center;font-family:sans-serif;background:#222;color:#fff;touch-action:none;} "
".slider{width:80%;height:50px;margin:20px;} .label{font-size:20px;}</style></head>"
"<body><h1>ESP Drone</h1>"
"<div class='label'>Throttle: <span id='t_val'>0</span>%</div>"
"<input type='range' min='0' max='100' value='0' class='slider' id='throttle' oninput='send()'>"
"<div class='label'>Steering: <span id='s_val'>0</span></div>"
"<input type='range' min='-50' max='50' value='0' class='slider' id='steering' oninput='send()'>"
"<br><button onclick='stop()' style='width:100px;height:50px;background:red;color:white;font-weight:bold;font-size:20px;'>STOP</button>"
"<script>"
"var ws = new WebSocket('ws://' + location.host + '/ws');"
"ws.onopen = function(){console.log('Connected');};"
"function send(){"
"  var t = document.getElementById('throttle').value;"
"  var s = document.getElementById('steering').value;"
"  document.getElementById('t_val').innerText = t;"
"  document.getElementById('s_val').innerText = s;"
"  if(ws.readyState === 1) ws.send(JSON.stringify({t: parseFloat(t), s: parseFloat(s)}));"
"}"
"function stop(){"
"  document.getElementById('throttle').value = 0;"
"  document.getElementById('steering').value = 0;"
"  send();"
"}"
"</script></body></html>";

/* --- HTTP Handler to serve Page --- */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, html_content, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* --- WebSocket Handler --- */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    //set max length to 128 bytes
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }
        
        //parse JSON
        cJSON *root = cJSON_Parse((char*)ws_pkt.payload);
        if (root) {
            cJSON *t = cJSON_GetObjectItem(root, "t");
            cJSON *s = cJSON_GetObjectItem(root, "s");
            if (t && s) {
                s_throttle = (float)t->valuedouble;
                s_steering = (float)s->valuedouble;
                s_new_data = 1;
            }
            cJSON_Delete(root);
        }
        free(buf);
    }
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t ws = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &ws);
        return server;
    }
    return NULL;
}

/* --- Wi-Fi Init --- */
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32-Drone",
            .ssid_len = strlen("ESP32-Drone"),
            .channel = 1,
            .password = "drone123",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s example",
             "ESP32-Drone", "drone123");
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

    wifi_init_softap();
    start_webserver();
}

int wifi_control_get_data(float *throttle, float *steering)
{
    *throttle = s_throttle;
    *steering = s_steering;
    int ret = s_new_data;
    s_new_data = 0;
    return ret;
}
