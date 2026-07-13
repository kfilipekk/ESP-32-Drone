#include "stabiliser.h"
#include <algorithm>

namespace drone {
namespace {

constexpr float dt_sec{0.001f};
constexpr float output_to_duty{100.0f / 65535.0f};
constexpr float min_active_throttle{5.0f};
constexpr float max_percent{100.0f};

constexpr float rate_rp_kp{250.0f};
constexpr float rate_rp_ki{500.0f};
constexpr float rate_rp_kd{2.5f};
constexpr float rate_rp_ilimit{33.3f};

constexpr float rate_yaw_kp{120.0f};
constexpr float rate_yaw_ki{16.7f};
constexpr float rate_yaw_kd{0.0f};
constexpr float rate_yaw_ilimit{166.7f};

constexpr float angle_rp_kp{5.9f};
constexpr float angle_rp_ki{0.0f};
constexpr float angle_rp_kd{0.0f};
constexpr float angle_rp_ilimit{20.0f};

float clamp_percent(const float value)
{
    return std::clamp(value, 0.0f, max_percent);
}

}

Stabiliser::Stabiliser()
    : angle_roll_{angle_rp_kp, angle_rp_ki, angle_rp_kd, angle_rp_ilimit, dt_sec},
      angle_pitch_{angle_rp_kp, angle_rp_ki, angle_rp_kd, angle_rp_ilimit, dt_sec},
      rate_roll_{rate_rp_kp, rate_rp_ki, rate_rp_kd, rate_rp_ilimit, dt_sec},
      rate_pitch_{rate_rp_kp, rate_rp_ki, rate_rp_kd, rate_rp_ilimit, dt_sec},
      rate_yaw_{rate_yaw_kp, rate_yaw_ki, rate_yaw_kd, rate_yaw_ilimit, dt_sec}
{
}

void Stabiliser::reset()
{
    angle_roll_.reset();
    angle_pitch_.reset();
    rate_roll_.reset();
    rate_pitch_.reset();
    rate_yaw_.reset();
}

void Stabiliser::tune(const std::int32_t target, const float kp, const float ki, const float kd)
{
    if (target == 0) {
        rate_roll_.set_gains(kp, ki, kd);
        rate_pitch_.set_gains(kp, ki, kd);
    } else if (target == 1) {
        rate_yaw_.set_gains(kp, ki, kd);
    } else if (target == 2) {
        angle_roll_.set_gains(kp, ki, kd);
        angle_pitch_.set_gains(kp, ki, kd);
    } else {
        //unknown target ignored
    }
}

void Stabiliser::run(const Attitude& attitude, const ControlCommand& command)
{
    const float target_roll_rate{angle_roll_.compute(command.roll, attitude.roll)};
    const float target_pitch_rate{angle_pitch_.compute(command.pitch, attitude.pitch)};

    float roll_output{rate_roll_.compute(target_roll_rate, attitude.roll_rate)};
    float pitch_output{rate_pitch_.compute(target_pitch_rate, attitude.pitch_rate)};
    float yaw_output{rate_yaw_.compute(command.yaw, attitude.yaw_rate)};

    //no correction or windup while idle so it cannot spin up tilted on the ground
    if (command.throttle < min_active_throttle) {
        reset();
        roll_output = 0.0f;
        pitch_output = 0.0f;
        yaw_output = 0.0f;
    }

    mix_motors(command.throttle, roll_output * output_to_duty, pitch_output * output_to_duty,
               yaw_output * output_to_duty);
}

void Stabiliser::mix_motors(const float throttle, const float roll, const float pitch, const float yaw)
{
    //quad x with front left and rear right spinning cw
    const float base{clamp_percent(throttle)};
    motor_power_[0U] = clamp_percent(((base + roll) + pitch) - yaw);
    motor_power_[1U] = clamp_percent(((base - roll) + pitch) + yaw);
    motor_power_[2U] = clamp_percent(((base - roll) - pitch) - yaw);
    motor_power_[3U] = clamp_percent(((base + roll) - pitch) + yaw);

    motor_set_all(motor_power_);
}

}
