#include "pid.h"
#include <algorithm>

namespace drone {

Pid::Pid(const float kp, const float ki, const float kd, const float integral_limit, const float dt)
    : kp_{kp}, ki_{ki}, kd_{kd}, integral_limit_{integral_limit}, dt_{dt}
{
}

void Pid::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void Pid::set_gains(const float kp, const float ki, const float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float Pid::compute(const float setpoint, const float measurement)
{
    const float error{setpoint - measurement};

    integral_ = std::clamp(integral_ + (error * dt_), -integral_limit_, integral_limit_);

    last_p_ = kp_ * error;
    last_d_ = kd_ * ((error - prev_error_) / dt_);
    prev_error_ = error;

    return last_p_ + (ki_ * integral_) + last_d_;
}

}
