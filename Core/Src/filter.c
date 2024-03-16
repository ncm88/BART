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
