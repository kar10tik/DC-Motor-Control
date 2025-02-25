#include "pid.h"
#include "motor_encoder.h"

extern volatile _Bool pid_compute_flag;
/**
 * @fn int16_t pid_pwm(Encoder_data)
 * @brief
 *
 * @param n20_encoder
 * @return
 */
void pid_pwm(Encoder_data n20_encoder, int32_t target_speed, TIM_HandleTypeDef* htim){

	static int32_t last_encoder_count = 0, prev_error = 0;
	int16_t error_integral = 0; //Integral error term

    int32_t count = n20_encoder.count;

    int32_t current_speed = (count - last_encoder_count)*30/PPR;
    last_encoder_count = count;

    int16_t error = target_speed - current_speed;
    int32_t p_term = KP * error;
    error_integral += error;

    if (error_integral > INTEGRAL_MAX) {
    	error_integral = INTEGRAL_MAX;
    }
    else if (error_integral < -INTEGRAL_MAX) {
      	error_integral = -INTEGRAL_MAX;
    }
    int32_t i_term = KI * error_integral;

    int32_t d_term = KD * (error - prev_error);
    prev_error = error;

    int32_t output = (p_term + i_term + d_term) / PID_SCALE;

    if (output > MAX_PWM) {
    	output = MAX_PWM;
    }
    else if (output < MIN_PWM) {
    	output = MIN_PWM;
    }
    htim->Instance->CCR1 = output;
    pid_compute_flag = 0;
}
