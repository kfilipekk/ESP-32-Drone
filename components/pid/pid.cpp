#include "pid.h"
#include <math.h>

Pid::Pid(float kp, float ki, float kd, float output_limit, float integral_limit, float dt)
    : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit), integral_limit_(integral_limit), dt_(dt)
{
    reset();
}

void Pid::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void Pid::set_gains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float Pid::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    float p_out = kp_ * error;

    integral_ += error * dt_;

    if (integral_ > integral_limit_) {
        integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
        integral_ = -integral_limit_;
    }

    float i_out = ki_ * integral_;

    float derivative = (error - prev_error_) / dt_;
    float d_out = kd_ * derivative;

    prev_error_ = error;

    float output = p_out + i_out + d_out;

    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }

    return output;
}
