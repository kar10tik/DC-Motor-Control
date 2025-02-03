#include "main.h"
#include "stdint.h"

#ifndef INC_MOTOR_ENCODER_H
#define INC_MOTOR_ENCODER_H

typedef struct{
	int32_t position;
	int32_t velocity;
	uint32_t prev_count;
} Encoder_data;

void update_encoder(Encoder_data *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(Encoder_data *encoder_value);

#endif
