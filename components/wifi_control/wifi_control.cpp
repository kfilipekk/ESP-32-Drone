#include "wifi_control.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <span>
#include <string_view>
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

namespace drone {
namespace {

constexpr const char* tag{"WIFI_CONTROL"};
constexpr const char* server_uri{"wss://krystianfilipek.com/ws?role=drone"};
constexpr int network_timeout_ms{5000};
constexpr std::uint8_t text_opcode{0x01U};
constexpr std::string_view digits{"0123456789"};

//fixed buffer json builder so telemetry needs no heap or printf
class JsonWriter {
public:
    void integer(const std::string_view key, const std::int32_t value)
    {
        begin(key);
        if (value < 0) {
            put('-');
        }
        put_unsigned(static_cast<std::uint32_t>(std::abs(static_cast<std::int64_t>(value))));
    }

    void tenths(const std::string_view key, const float value) { scaled(key, value, 10U); }
    void whole(const std::string_view key, const float value) { scaled(key, value, 1U); }

    void close() { put('}'); }
    bool valid() const { return !overflow_; }
    const char* data() const { return buffer_.data(); }
    int size() const { return static_cast<int>(length_); }

private:
    void begin(const std::string_view key)
    {
        put((length_ == 0U) ? '{' : ',');
        put('"');
        for (const char c : key) {
            put(c);
        }
        put('"');
        put(':');
    }

    void scaled(const std::string_view key, const float value, const std::uint32_t scale)
    {
        begin(key);
        const float magnitude{std::isfinite(value) ? std::round(std::fabs(value) * static_cast<float>(scale)) : 0.0f};
        const auto units{static_cast<std::uint32_t>(std::fmin(magnitude, 4.0e9f))};
        if ((value < 0.0f) && (units > 0U)) {
            put('-');
        }
        put_unsigned(units / scale);
        if (scale > 1U) {
            put('.');
            put(digits[units % scale]);
        }
    }

    void put_unsigned(const std::uint32_t value)
    {
        std::array<char, 10U> reversed{};
        std::size_t count{0U};
        std::uint32_t rest{value};
        do {
            reversed[count] = digits[rest % 10U];
            ++count;
            rest /= 10U;
        } while (rest > 0U);
        while (count > 0U) {
            --count;
            put(reversed[count]);
        }
    }

    void put(const char c)
    {
        if (length_ < buffer_.size()) {
            buffer_[length_] = c;
            ++length_;
        } else {
            overflow_ = true;
        }
    }

    std::array<char, 384U> buffer_{};
    std::size_t length_{0U};
    bool overflow_{false};
};

void read_number(const cJSON* const root, const char* const short_key, const char* const long_key, float& target)
{
    const cJSON* item{cJSON_GetObjectItem(root, short_key)};
    if (item == nullptr) {
        item = cJSON_GetObjectItem(root, long_key);
    }
    if (item != nullptr) {
        target = static_cast<float>(item->valuedouble);
    }
}

void copy_text(const std::string_view text, const std::span<std::uint8_t> target)
{
    const std::size_t count{(text.size() < target.size()) ? text.size() : target.size()};
    for (std::size_t i{0U}; i < count; ++i) {
        target[i] = static_cast<std::uint8_t>(text[i]);
    }
}

}

void WifiControl::init()
{
    esp_err_t ret{nvs_flash_init()};
    if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    static_cast<void>(esp_netif_create_default_wifi_sta());

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, this, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, static_cast<std::int32_t>(IP_EVENT_STA_GOT_IP),
                                                        &on_wifi_event, this, nullptr));

    wifi_config_t wifi_config{};
    copy_text(CONFIG_WIFI_SSID, wifi_config.sta.ssid);
    copy_text(CONFIG_WIFI_PASSWORD, wifi_config.sta.password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //power save off for lower latency
    static_cast<void>(esp_wifi_set_ps(WIFI_PS_NONE));
}

bool WifiControl::poll(RemoteCommand& command)
{
    const std::lock_guard<std::mutex> lock{mutex_};
    const bool fresh{fresh_};
    command = latest_;
    latest_.has_tuning = false;
    fresh_ = false;
    return fresh;
}

void WifiControl::send(const Telemetry& telemetry) const
{
    const esp_websocket_client_handle_t client{client_.load()};
    if ((client == nullptr) || !esp_websocket_client_is_connected(client)) {
        return;
    }

    //short keys keep the packet small, t 1 marks telemetry
    JsonWriter json{};
    json.integer("t", 1);
    json.tenths("r", telemetry.roll);
    json.tenths("p", telemetry.pitch);
    json.tenths("y", telemetry.yaw);
    json.tenths("v", telemetry.voltage);
    json.integer("ax", telemetry.ax);
    json.integer("ay", telemetry.ay);
    json.integer("az", telemetry.az);
    json.integer("gx", telemetry.gx);
    json.integer("gy", telemetry.gy);
    json.integer("gz", telemetry.gz);
    json.whole("m1", telemetry.motors[0U]);
    json.whole("m2", telemetry.motors[1U]);
    json.whole("m3", telemetry.motors[2U]);
    json.whole("m4", telemetry.motors[3U]);
    json.tenths("pi", telemetry.p_term);
    json.tenths("ii", telemetry.i_term);
    json.tenths("di", telemetry.d_term);
    json.close();

    if (json.valid()) {
        static_cast<void>(esp_websocket_client_send_text(client, json.data(), json.size(), 0U));
    }
}

void WifiControl::on_wifi_event(void* const arg, const esp_event_base_t base, const std::int32_t id, void*)
{
    if ((base == WIFI_EVENT) && (id == static_cast<std::int32_t>(WIFI_EVENT_STA_START))) {
        static_cast<void>(esp_wifi_connect());
    } else if ((base == WIFI_EVENT) && (id == static_cast<std::int32_t>(WIFI_EVENT_STA_DISCONNECTED))) {
        ESP_LOGI(tag, "Disconnected. Retrying");
        static_cast<void>(esp_wifi_connect());
    } else if ((base == IP_EVENT) && (id == static_cast<std::int32_t>(IP_EVENT_STA_GOT_IP))) {
        ESP_LOGI(tag, "Drone connected");
        static_cast<WifiControl*>(arg)->start_websocket();
    } else {
        //other events unused
    }
}

void WifiControl::on_websocket_event(void* const arg, esp_event_base_t, const std::int32_t id, void* const data)
{
    if (id == static_cast<std::int32_t>(WEBSOCKET_EVENT_CONNECTED)) {
        ESP_LOGI(tag, "Websocket connected");
    } else if (id == static_cast<std::int32_t>(WEBSOCKET_EVENT_DISCONNECTED)) {
        ESP_LOGI(tag, "Websocket disconnected");
    } else if (id == static_cast<std::int32_t>(WEBSOCKET_EVENT_ERROR)) {
        ESP_LOGE(tag, "Websocket error");
    } else if (id == static_cast<std::int32_t>(WEBSOCKET_EVENT_DATA)) {
        const auto* const event{static_cast<const esp_websocket_event_data_t*>(data)};
        if ((event->op_code == text_opcode) && (event->data_len > 0)) {
            static_cast<WifiControl*>(arg)->parse(event->data_ptr, event->data_len);
        }
    } else {
        //other events unused
    }
}

void WifiControl::start_websocket()
{
    //client reconnects by itself so only create it once
    if (client_.load() != nullptr) {
        return;
    }

    esp_websocket_client_config_t config{};
    config.uri = server_uri;
    config.transport = WEBSOCKET_TRANSPORT_OVER_SSL;
    config.crt_bundle_attach = &esp_crt_bundle_attach;
    config.network_timeout_ms = network_timeout_ms;

    const esp_websocket_client_handle_t client{esp_websocket_client_init(&config)};
    if (client == nullptr) {
        ESP_LOGE(tag, "Websocket init failed");
        return;
    }

    static_cast<void>(esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, &on_websocket_event, this));
    if (esp_websocket_client_start(client) == ESP_OK) {
        client_.store(client);
        ESP_LOGI(tag, "Connecting to %s", server_uri);
    } else {
        ESP_LOGE(tag, "Websocket start failed");
        static_cast<void>(esp_websocket_client_destroy(client));
    }
}

void WifiControl::parse(const char* const text, const int length)
{
    cJSON* const root{cJSON_ParseWithLength(text, static_cast<std::size_t>(length))};
    if (root == nullptr) {
        return;
    }

    {
        const std::lock_guard<std::mutex> lock{mutex_};
        read_number(root, "t", "throttle", latest_.throttle);
        read_number(root, "r", "roll", latest_.roll);
        read_number(root, "p", "pitch", latest_.pitch);
        read_number(root, "y", "yaw", latest_.yaw);

        //tuning packet {"tid":0,"kp":1.0,"ki":0.0,"kd":0.0}
        const cJSON* const tid{cJSON_GetObjectItem(root, "tid")};
        const cJSON* const kp{cJSON_GetObjectItem(root, "kp")};
        const cJSON* const ki{cJSON_GetObjectItem(root, "ki")};
        const cJSON* const kd{cJSON_GetObjectItem(root, "kd")};
        if ((tid != nullptr) && (kp != nullptr) && (ki != nullptr) && (kd != nullptr)) {
            latest_.has_tuning = true;
            latest_.tuning_id = static_cast<std::int32_t>(tid->valueint);
            latest_.kp = static_cast<float>(kp->valuedouble);
            latest_.ki = static_cast<float>(ki->valuedouble);
            latest_.kd = static_cast<float>(kd->valuedouble);
        }
        fresh_ = true;
    }

    cJSON_Delete(root);
}

}
