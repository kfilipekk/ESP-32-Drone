#pragma once

class Kalman {
public:
    Kalman();

    void set_angle(float angle);
    float get_angle(float new_angle, float new_rate, float dt);
    float get_rate();

    void set_q_angle(float q_angle);
    void set_q_bias(float q_bias);
    void set_r_measure(float r_measure);

private:
    float q_angle_;
    float q_bias_;
    float r_measure_;

    float angle_;
    float bias_;
    float rate_;

    float p_[2][2];
};
