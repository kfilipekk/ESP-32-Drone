#ifndef MOTOR_H
#define MOTOR_H

#include <array>
#include <cstddef>

namespace drone {

constexpr std::size_t motor_count{4U};

//duty in percent ordered front left, front right, rear right, rear left
using MotorDuty = std::array<float, motor_count>;

void motor_init();
void motor_set_all(const MotorDuty& duty);

}

#endif
