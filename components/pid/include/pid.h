#ifndef PID_H
#define PID_H

namespace drone {

class Pid {
public:
    Pid(float kp, float ki, float kd, float integral_limit, float dt);

    float compute(float setpoint, float measurement);
    void reset();
    void set_gains(float kp, float ki, float kd);

    //debug terms for telemetry
    float last_p() const { return last_p_; }
    float integral() const { return integral_; }
    float last_d() const { return last_d_; }

private:
    float kp_;
    float ki_;
    float kd_;
    float integral_limit_;
    float dt_;
    float integral_{0.0f};
    float prev_error_{0.0f};
    float last_p_{0.0f};
    float last_d_{0.0f};
};

}

#endif
