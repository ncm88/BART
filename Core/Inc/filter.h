#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"


#define MOVING_AVERAGE_LENGTH  30
#define EMA_LENGTH 10
#define RN_VALUES_10 {1.0, 0.6411803884299546, 0.41111229050718745, 0.26359713811572677, 0.1690133154060661, 0.10836802322189586, 0.06948345122280154, 0.04455142624448969, 0.02856550078455038, 0.01831563888873418}


typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int32_t sum;
	float out;
} moving_avg_obj;

typedef struct{
	float buffer[EMA_LENGTH];
	uint16_t counter;
	float sum;
	float out;
} ema_obj;


void reset_average_filter(moving_avg_obj* instance);
void apply_average_filter(moving_avg_obj* instance, int16_t input, float* out);
void apply_average_filter_unsigned(moving_avg_obj* instance, uint16_t input, uint16_t* out);

void apply_ema_filter(ema_obj* instance, uint16_t input, float* out);
void reset_ema_filter(ema_obj* instance);



#endif /* INC_MOVING_AVERAGE_INT16_H_ */