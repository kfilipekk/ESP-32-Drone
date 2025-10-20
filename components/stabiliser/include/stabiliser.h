#pragma once

#include "pid.h"

typedef enum {
    STABILISER_MODE_RATE,
    STABILISER_MODE_ANGLE
} stabiliser_mode_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} control_command_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} state_estimate_t;

class Stabiliser {
public:
    Stabiliser();

    void init();

    void run(state_estimate_t state, control_command_t command);

    float get_motor_output(int index) { return motor_power_[index]; }

    void set_mode(stabiliser_mode_t mode) { current_mode_ = mode; }
    
    //PID getters/setters
    Pid* get_pid_rate_roll() { return &pid_rate_roll_; }
    Pid* get_pid_rate_pitch() { return &pid_rate_pitch_; }
    Pid* get_pid_rate_yaw() { return &pid_rate_yaw_; }
    Pid* get_pid_angle_roll() { return &pid_angle_roll_; }
    Pid* get_pid_angle_pitch() { return &pid_angle_pitch_; }

private:
    void mix_motors(float throttle, float roll, float pitch, float yaw);

    stabiliser_mode_t current_mode_;

    //inner loop (rate) controllers
    Pid pid_rate_roll_;
    Pid pid_rate_pitch_;
    Pid pid_rate_yaw_;

    //outer loop (angle) controllers
    Pid pid_angle_roll_;
    Pid pid_angle_pitch_;

    float motor_power_[4];
    
};