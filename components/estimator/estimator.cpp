#include "estimator.h"
#include <cmath>

namespace drone {
namespace {

constexpr float rad_to_deg{57.2957795131f};
constexpr float gyro_lsb_per_dps{65.5f};

constexpr float q_angle{0.001f};
constexpr float q_bias{0.003f};
constexpr float r_measure{0.03f};

}

float Kalman::update(const float measured_angle, const float rate, const float dt)
{
    angle_ += dt * (rate - bias_);

    p_[0U][0U] += dt * (((dt * p_[1U][1U]) - p_[0U][1U]) - p_[1U][0U] + q_angle);
    p_[0U][1U] -= dt * p_[1U][1U];
    p_[1U][0U] -= dt * p_[1U][1U];
    p_[1U][1U] += q_bias * dt;

    const float s{p_[0U][0U] + r_measure};
    const float k_0{p_[0U][0U] / s};
    const float k_1{p_[1U][0U] / s};
    const float y{measured_angle - angle_};

    angle_ += k_0 * y;
    bias_ += k_1 * y;

    const float p00{p_[0U][0U]};
    const float p01{p_[0U][1U]};

    p_[0U][0U] -= k_0 * p00;
    p_[0U][1U] -= k_0 * p01;
    p_[1U][0U] -= k_1 * p00;
    p_[1U][1U] -= k_1 * p01;

    return angle_;
}

void Estimator::update(const ImuSample& imu, const float dt)
{
    //sensor is rotated on the board so swap x/y and flip the gyro axes
    const float acc_x{static_cast<float>(imu.accel_y)};
    const float acc_y{static_cast<float>(imu.accel_x)};
    const float acc_z{static_cast<float>(imu.accel_z)};

    const float rate_roll{-static_cast<float>(imu.gyro_y) / gyro_lsb_per_dps};
    const float rate_pitch{-static_cast<float>(imu.gyro_x) / gyro_lsb_per_dps};
    const float rate_yaw{-static_cast<float>(imu.gyro_z) / gyro_lsb_per_dps};

    const float acc_roll{std::atan2(acc_y, acc_z) * rad_to_deg};
    const float acc_pitch{std::atan2(-acc_x, std::sqrt((acc_y * acc_y) + (acc_z * acc_z))) * rad_to_deg};

    attitude_.roll = roll_filter_.update(acc_roll, rate_roll, dt);
    attitude_.pitch = pitch_filter_.update(acc_pitch, rate_pitch, dt);
    attitude_.yaw += rate_yaw * dt;
    attitude_.roll_rate = rate_roll;
    attitude_.pitch_rate = rate_pitch;
    attitude_.yaw_rate = rate_yaw;
}

}
