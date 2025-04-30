#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_M1_GPIO   5  //front Left
#define MOTOR_M2_GPIO   6  //front Right
#define MOTOR_M3_GPIO   3  //rear Right
#define MOTOR_M4_GPIO   4  //rear Left

//Initialise the motor driver (LEDC PWM)
void motor_init(void);

//set thrust for a specific motor
void motor_set_thrust(int motor_id, float duty_percent);

//set thrust for all motors
void motor_set_all(float m1, float m2, float m3, float m4);

void motor_stop_all(void);

#ifdef __cplusplus
}
#endif
