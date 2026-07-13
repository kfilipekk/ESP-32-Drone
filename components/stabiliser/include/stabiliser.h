#ifndef STABILISER_H
#define STABILISER_H

#include <cstdint>
#include "estimator.h"
#include "motor.h"
#include "pid.h"

namespace drone {

//setpoints in degrees, throttle in percent
struct ControlCommand {
    float roll;
    float pitch;
    float yaw;
    float throttle;
};

class Stabiliser {
public:
    Stabiliser();

    void run(const Attitude& attitude, const ControlCommand& command);

    //target 0 rate roll/pitch, 1 rate yaw, 2 angle roll/pitch
    void tune(std::int32_t target, float kp, float ki, float kd);

    const MotorDuty& motor_outputs() const { return motor_power_; }
    const Pid& rate_pitch() const { return rate_pitch_; }

private:
    void reset();
    void mix_motors(float throttle, float roll, float pitch, float yaw);

    //outer loop angle controllers feed the inner loop rate controllers
    Pid angle_roll_;
    Pid angle_pitch_;
    Pid rate_roll_;
    Pid rate_pitch_;
    Pid rate_yaw_;
    MotorDuty motor_power_{};
};

}

#endif
