#pragma once

#include "mpu6050.h"
#include "kalman.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} estimated_state_t;

class Estimator {
public:
    Estimator();
    
    void init();
    
    void update(mpu6050_data_t& imu_data, float dt);
    
    estimated_state_t get_state() const { return state_; }

private:
    Kalman kalman_roll_;
    Kalman kalman_pitch_;

    estimated_state_t state_;
};
