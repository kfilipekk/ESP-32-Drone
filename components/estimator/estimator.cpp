#include "estimator.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795131f

Estimator::Estimator() {
    state_ = {0};
}

void Estimator::init() {
}

void Estimator::update(mpu6050_data_t& imu_data, float dt) {
    //sensor mapping logic for rotated board
    float logical_acc_x = imu_data.accel_y;
    float logical_acc_y = imu_data.accel_x;
    float logical_acc_z = imu_data.accel_z;

    float logical_gyro_x = imu_data.gyro_y;
    //invert pitch rate because sensor is flipped
    float logical_gyro_y = -imu_data.gyro_x;
    //invert yaw rate to match frame
    float logical_gyro_z = -imu_data.gyro_z; 

    logical_gyro_x = -imu_data.gyro_y; 

    float acc_roll = atan2f(logical_acc_y, logical_acc_z) * RAD_TO_DEG;
    float acc_pitch = atan2f(-logical_acc_x, sqrtf(powf(logical_acc_y, 2) + powf(logical_acc_z, 2))) * RAD_TO_DEG;

    float gyro_scale = 65.5f; 
    float gyro_rate_roll = logical_gyro_x / gyro_scale;
    float gyro_rate_pitch = logical_gyro_y / gyro_scale;
    float gyro_rate_yaw = logical_gyro_z / gyro_scale;

    state_.roll = kalman_roll_.get_angle(acc_roll, gyro_rate_roll, dt);
    state_.pitch = kalman_pitch_.get_angle(acc_pitch, gyro_rate_pitch, dt);
    
    state_.yaw += gyro_rate_yaw * dt;

    state_.roll_rate = gyro_rate_roll;
    state_.pitch_rate = gyro_rate_pitch;
    state_.yaw_rate = gyro_rate_yaw;
}
