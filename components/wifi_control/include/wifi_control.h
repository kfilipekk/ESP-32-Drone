#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include "esp_event.h"
#include "esp_websocket_client.h"

namespace drone {

struct RemoteCommand {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    bool has_tuning;
    std::int32_t tuning_id;
    float kp;
    float ki;
    float kd;
};

struct Telemetry {
    float roll;
    float pitch;
    float yaw;
    float voltage;
    std::int16_t ax;
    std::int16_t ay;
    std::int16_t az;
    std::int16_t gx;
    std::int16_t gy;
    std::int16_t gz;
    std::array<float, 4U> motors;
    float p_term;
    float i_term;
    float d_term;
};

class WifiControl {
public:
    void init();
    bool poll(RemoteCommand& command);
    void send(const Telemetry& telemetry) const;

private:
    static void on_wifi_event(void* arg, esp_event_base_t base, std::int32_t id, void*);
    static void on_websocket_event(void* arg, esp_event_base_t, std::int32_t id, void* data);
    void start_websocket();
    void parse(const char* text, int length);

    std::mutex mutex_;
    RemoteCommand latest_{};
    bool fresh_{false};
    std::atomic<esp_websocket_client_handle_t> client_{nullptr};
};

}

#endif
