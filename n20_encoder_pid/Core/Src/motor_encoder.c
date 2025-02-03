#include "motor_encoder.h"


void update_encoder(Encoder_data *encoder_value, TIM_HandleTypeDef *htim){
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim); //htim->Instance->CNT
	if (temp_counter > encoder_value->prev_count){
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
			//Handle counter overflow condition
			encoder_value->velocity = -(encoder_value->prev_count) - htim->Instance->ARR - temp_counter;
		}
		else{
			encoder_value->velocity = temp_counter - encoder_value->prev_count;
		}
	}
	else if (temp_counter < encoder_value->prev_count){
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
			encoder_value->velocity = temp_counter - encoder_value->prev_count;
		}
		else{
			encoder_value->velocity = temp_counter + htim->Instance->ARR - (encoder_value->prev_count);
		}
	}
	encoder_value-> position += encoder_value->velocity;
	encoder_value->prev_count = temp_counter;
	return;
}


void reset_encoder(Encoder_data *encoder_value){
	encoder_value->position = 0;
	encoder_value->prev_count=0;
	encoder_value->velocity = 0;
}
