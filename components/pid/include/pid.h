#pragma once

class Pid {
public:
    Pid(float kp, float ki, float kd, float output_limit, float integral_limit, float dt);

    float compute(float setpoint, float measurement);

    /**
     * @brief reset internal PID state (integral, prev_error)
     */
    void reset();

    void set_gains(float kp, float ki, float kd);

    float integral_;
    float prev_error_;
    
    float output_limit_;
    float integral_limit_;
    float dt_;

    //debug storage
    float last_p_ = 0.0f;
    float last_d_ = 0.0f;
};
