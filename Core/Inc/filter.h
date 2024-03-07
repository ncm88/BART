#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

#define MOVING_AVERAGE_LENGTH  100
typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int16_t out;
	int32_t sum;
} filterObj;

void reset_average_filter(filterObj* instance);
void apply_average_filter(filterObj* instance, int16_t input, int16_t *out);

#endif /* INC_MOVING_AVERAGE_INT16_H_ */