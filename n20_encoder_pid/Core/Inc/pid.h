#include "main.h"
#include "motor_encoder.h"

#ifndef INC_PID_H
#define INC_PID_H

void pid_pwm(Encoder_data encoder_value, int32_t target_speed, TIM_HandleTypeDef* htim);
#endif
