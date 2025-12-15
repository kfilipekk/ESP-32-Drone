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

    float get_kp() const { return kp_; }
    float get_ki() const { return ki_; }
    float get_kd() const { return kd_; }
    
    //debug getters
    float get_last_p() const { return last_p_; }
    float get_last_i() const { return integral_; } //integral storage
    float get_last_d() const { return last_d_; }

private:
    float kp_;
    float ki_;
    float kd_;
    
    float integral_;
    float prev_error_;
    
    float output_limit_;
    float integral_limit_;
    float dt_;
    //debug storage
    float last_p_ = 0.0f;
    float last_d_ = 0.0f;
};
