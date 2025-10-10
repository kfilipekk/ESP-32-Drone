#include "kalman.h"

Kalman::Kalman() {
    q_angle_ = 0.001f;
    q_bias_ = 0.003f;
    r_measure_ = 0.03f;

    angle_ = 0.0f;
    bias_ = 0.0f;
    rate_ = 0.0f;

    p_[0][0] = 0.0f;
    p_[0][1] = 0.0f;
    p_[1][0] = 0.0f;
    p_[1][1] = 0.0f;
}

void Kalman::set_angle(float angle) {
    angle_ = angle;
}

float Kalman::get_rate() {
    return rate_;
}

void Kalman::set_q_angle(float q_angle) { q_angle_ = q_angle; }
void Kalman::set_q_bias(float q_bias) { q_bias_ = q_bias; }
void Kalman::set_r_measure(float r_measure) { r_measure_ = r_measure; }

float Kalman::get_angle(float new_angle, float new_rate, float dt) {
    rate_ = new_rate - bias_;
    angle_ += dt * rate_;

    p_[0][0] += dt * (dt * p_[1][1] - p_[0][1] - p_[1][0] + q_angle_);
    p_[0][1] -= dt * p_[1][1];
    p_[1][0] -= dt * p_[1][1];
    p_[1][1] += q_bias_ * dt;

    float s = p_[0][0] + r_measure_;
    float k_0 = p_[0][0] / s;
    float k_1 = p_[1][0] / s;

    float y = new_angle - angle_;

    angle_ += k_0 * y;
    bias_ += k_1 * y;

    float p00_temp = p_[0][0];
    float p01_temp = p_[0][1];

    p_[0][0] -= k_0 * p00_temp;
    p_[0][1] -= k_0 * p01_temp;
    p_[1][0] -= k_1 * p00_temp;
    p_[1][1] -= k_1 * p01_temp;

    return angle_;
}
