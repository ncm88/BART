#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

#define MOVING_AVERAGE_LENGTH  100
typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int16_t out;
	int32_t sum;
}mov_aver_intance_int16;

void reset_average_filter(mov_aver_intance_int16* instance);
void apply_average_filter(mov_aver_intance_int16* instance, int16_t input, int16_t *out);

#endif /* INC_MOVING_AVERAGE_INT16_H_ */