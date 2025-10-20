#include "stabiliser.h"
#include "motor.h"
#include <algorithm>

#define DT_SEC 0.001f

#define CF_TO_DUTY_SCALE (100.0f / 65535.0f)
#define RATE_RP_KP  250.0f
#define RATE_RP_KI  500.0f
#define RATE_RP_KD  2.5f
#define RATE_RP_ILIMIT 33.3f

#define RATE_YAW_KP 120.0f
#define RATE_YAW_KI 16.7f
#define RATE_YAW_KD 0.0f
#define RATE_YAW_ILIMIT 166.7f


#define ANGLE_RP_KP 5.9f
#define ANGLE_RP_KI 0.0f
#define ANGLE_RP_KD 0.0f
#define ANGLE_RP_ILIMIT 20.0f

#define ANGLE_YAW_KP 6.0f
#define ANGLE_YAW_KI 1.0f
#define ANGLE_YAW_KD 0.35f
#define ANGLE_YAW_ILIMIT 360.0f

#define NO_LIMIT 0.0f

Stabiliser::Stabiliser()
    : current_mode_(STABILISER_MODE_ANGLE),
      pid_rate_roll_(RATE_RP_KP, RATE_RP_KI, RATE_RP_KD, NO_LIMIT, RATE_RP_ILIMIT, DT_SEC),
      pid_rate_pitch_(RATE_RP_KP, RATE_RP_KI, RATE_RP_KD, NO_LIMIT, RATE_RP_ILIMIT, DT_SEC),
      pid_rate_yaw_(RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, NO_LIMIT, RATE_YAW_ILIMIT, DT_SEC),
      pid_angle_roll_(ANGLE_RP_KP, ANGLE_RP_KI, ANGLE_RP_KD, NO_LIMIT, ANGLE_RP_ILIMIT, DT_SEC),
      pid_angle_pitch_(ANGLE_RP_KP, ANGLE_RP_KI, ANGLE_RP_KD, NO_LIMIT, ANGLE_RP_ILIMIT, DT_SEC)
{
}

void Stabiliser::init() {
    pid_rate_roll_.reset();
    pid_rate_pitch_.reset();
    pid_rate_yaw_.reset();
    pid_angle_roll_.reset();
    pid_angle_pitch_.reset();
}

void Stabiliser::run(state_estimate_t state, control_command_t command) {
    float target_rate_roll = 0.0f;
    float target_rate_pitch = 0.0f;
    float target_rate_yaw = command.yaw;
    
    if (current_mode_ == STABILISER_MODE_ANGLE) {
        target_rate_roll = pid_angle_roll_.compute(command.roll, state.roll);
        target_rate_pitch = pid_angle_pitch_.compute(command.pitch, state.pitch);
    } else {
        target_rate_roll = command.roll;
        target_rate_pitch = command.pitch;
    }

    float roll_output = pid_rate_roll_.compute(target_rate_roll, state.roll_rate);
    float pitch_output = pid_rate_pitch_.compute(target_rate_pitch, state.pitch_rate);
    float yaw_output = pid_rate_yaw_.compute(target_rate_yaw, state.yaw_rate);

    //safety: if throttle is low, disable i-term windup and zero output
    //this prevents the drone from spinning up motors when sitting on the ground tilted
    if (command.throttle < 5.0f) {
        pid_rate_roll_.reset();
        pid_rate_pitch_.reset();
        pid_rate_yaw_.reset();
        pid_angle_roll_.reset();
        pid_angle_pitch_.reset();
        
        roll_output = 0;
        pitch_output = 0;
        yaw_output = 0;
    }

    roll_output *= CF_TO_DUTY_SCALE;
    pitch_output *= CF_TO_DUTY_SCALE;
    yaw_output *= CF_TO_DUTY_SCALE;

    mix_motors(command.throttle, roll_output, pitch_output, yaw_output);
}

void Stabiliser::mix_motors(float throttle, float roll, float pitch, float yaw) {
    if(throttle < 0) throttle = 0;
    if(throttle > 100) throttle = 100;

    //normal quad x configuration
    //m1: front left (cw)
    //m2: front right (ccw)
    //m3: rear right (cw)
    //m4: rear left (ccw)

    //mixing logic for torque and thrust
    float m1 = throttle + roll + pitch - yaw; //fl
    float m2 = throttle - roll + pitch + yaw; //fr
    float m3 = throttle - roll - pitch - yaw; //rr
    float m4 = throttle + roll - pitch + yaw; //rl

    auto clamp_motor = [](float val) -> float {
        if(val < 0.0f) return 0.0f;
        if(val > 100.0f) return 100.0f;
        return val;
    };

    motor_power_[0] = clamp_motor(m1);
    motor_power_[1] = clamp_motor(m2);
    motor_power_[2] = clamp_motor(m3);
    motor_power_[3] = clamp_motor(m4);

    motor_set_all(motor_power_[0], motor_power_[1], motor_power_[2], motor_power_[3]);
}
