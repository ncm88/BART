#include "filter.h"



void reset_average_filter(moving_avg_obj* instance)
{
	instance->counter = 0;
	instance->sum = 0;
	instance->out = 0;
	for (int i = 0; i < MOVING_AVERAGE_LENGTH; i++){
		instance->buffer[i] = 0;
	}
}


void reset_ema_filter(ema_obj* instance)
{
	instance->sum = 0;
	for (int i = 0; i < EMA_LENGTH; i++){
		instance->buffer[i] = 0;
	}
}



void apply_average_filter(moving_avg_obj* instance, int16_t input, float* out)
{
	static int16_t count = 0;
    if(count < MOVING_AVERAGE_LENGTH) count++;

    instance->sum += input - instance->buffer[instance->counter];
	instance->buffer[instance->counter] = input;
	instance->counter++;
	
    if(instance->counter == MOVING_AVERAGE_LENGTH) instance->counter = 0;
	
	instance->out = (float)instance->sum / count;
	
    // normalization
	*out = instance->out;
}



inline void apply_average_filter_unsigned(moving_avg_obj* instance, uint16_t input, uint16_t* out)
{
	static int16_t count = 0;
    if(count < MOVING_AVERAGE_LENGTH) count++;

    instance->sum += input - instance->buffer[instance->counter];
	instance->buffer[instance->counter] = input;
	instance->counter++;
	
    if(instance->counter == MOVING_AVERAGE_LENGTH) instance->counter = 0;
	
	instance->out = instance->sum / count;
	
    // normalization
	*out = instance->out;
}



void apply_ema_filter(ema_obj* instance, uint16_t input, float* out){
	static double Rn[EMA_LENGTH] = RN_VALUES_10;
	static uint8_t count = 0;
	if(count<EMA_LENGTH) count++;
	
	instance->sum = 0;
	
	for(uint8_t i = 1; i < count; i++){
		instance->buffer[i - 1] = instance->buffer[i];
		instance->sum += instance->buffer[i - 1] * Rn[i - 1];
	}

	instance->buffer[count - 1] = input;
	instance->sum += input * Rn[count - 1];
	*out = instance->sum / count;
}