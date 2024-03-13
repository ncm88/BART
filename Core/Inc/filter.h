#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"


#define MOVING_AVERAGE_LENGTH  10

typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int32_t sum;
	float out;
} moving_avg_obj;

void reset_average_filter(moving_avg_obj* instance);
void apply_average_filter(moving_avg_obj* instance, int16_t input, float* out);
void apply_average_filter_unsigned(moving_avg_obj* instance, uint16_t input, uint16_t* out);

#endif /* INC_MOVING_AVERAGE_INT16_H_ */