#include "encoder.h"


void update_encoder(encoder_instance* handle, TIM_HandleTypeDef* htim)
{
	uint32_t current_pos = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 1;
	
    if(first_time)
	{
		handle->velocity = 0;
		first_time = 0;
	}
	
    else{
        if(current_pos == handle->prevPos) handle->velocity = 0;
		
		else if(current_pos > handle->prevPos){
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) handle->velocity = current_pos - (handle->prevPos + __HAL_TIM_GetAutoreload(htim));
            else handle->velocity = current_pos - handle->prevPos;
		}

		else{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) handle->velocity = current_pos - handle->prevPos;
			else handle->velocity = current_pos + (__HAL_TIM_GET_AUTORELOAD(htim) - handle->prevPos);
		}
	}

	handle->position += handle->velocity;
	handle->prevPos = current_pos;
}

void reset_encoder(encoder_instance *handle)
{
	handle->velocity = 0;
	handle->position = 0;
	handle->prevPos = 0;
}
