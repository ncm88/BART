#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

#define MOVING_AVERAGE_LENGTH  20
typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int16_t out;
	int32_t sum;
} moving_avg_obj;

void reset_average_filter(moving_avg_obj* instance);
void apply_average_filter(moving_avg_obj* instance, int16_t input, int16_t* out);
void apply_average_filter_unsigned(moving_avg_obj* instance, uint16_t input, uint16_t* out);

#endif /* INC_MOVING_AVERAGE_INT16_H_ */