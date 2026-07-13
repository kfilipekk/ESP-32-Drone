#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <array>
#include "mpu6050.h"

namespace drone {

//angles in degrees, rates in degrees per second
struct Attitude {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
};

class Kalman {
public:
    float update(float measured_angle, float rate, float dt);

private:
    float angle_{0.0f};
    float bias_{0.0f};
    std::array<std::array<float, 2U>, 2U> p_{};
};

class Estimator {
public:
    void update(const ImuSample& imu, float dt);
    const Attitude& attitude() const { return attitude_; }

private:
    Kalman roll_filter_;
    Kalman pitch_filter_;
    Attitude attitude_{};
};

}

#endif
