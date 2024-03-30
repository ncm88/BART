#ifndef PID_H
#define PID_H

#include<stdint.h>

typedef struct
{
	float p_gain;
	float d_gain;
	int16_t output;
}pd_instance_int16;

void set_pd_gain(pd_instance_int16 *pd_instance, float p, float d);
void apply_pd(pd_instance_int16* pd_instance, int32_t input_error, float delta_filtered, uint16_t sampling_rate);

#endif /* INC_MOTOR_H_ */
