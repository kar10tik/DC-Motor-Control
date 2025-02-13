#include "motor_encoder.h"

/**
 * @fn void update_encoder(Encoder_data*, TIM_HandleTypeDef*)
 * @brief
 *
 * @param encoder_value
 * @param htim
 */
void update_encoder(Encoder_data *encoder_value, TIM_HandleTypeDef *htim){
	uint16_t temp_counter = htim->Instance->CNT; //__HAL_TIM_GET_COUNTER(htim)
	int16_t delta_count = 0;
	if (temp_counter > encoder_value->count){
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
			//Handle counter overflow condition
			delta_count = -(encoder_value->count) - htim->Instance->ARR - temp_counter;
			}
		else{
			delta_count = temp_counter - encoder_value->count;
			}
		}

	else if (temp_counter < encoder_value->count){
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
			delta_count = temp_counter - encoder_value->count;
		}
		else{
			delta_count = temp_counter + htim->Instance->ARR - (encoder_value->count);
			}
		}

		// Store calculated velocity
	encoder_value->velocity = delta_count;
		// Update absolute position
	encoder_value->position += encoder_value->velocity;
		// Store last encoder count
	encoder_value->count = temp_counter;
}

/**
 * @fn void reset_encoder(Encoder_data*)
 * @brief
 *
 * @param encoder_value
 */
void reset_encoder(Encoder_data *encoder_value){
	encoder_value->position = 0;
	encoder_value->count = 0;
	encoder_value->velocity = 0;
}
