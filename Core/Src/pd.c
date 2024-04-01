#include "pd.h"

#define PD_MAX  7998
#define RESOLUTION 48960
#define SCALE_FACTOR 5

#define ABS(A) (A >= 0)? (A) : ((-1) * (A))

void set_pd_gain(pd_instance_int16* pd, float p, float d)
{
	pd->p_gain = p;
	pd->d_gain = d;
}

void reset_pd(pd_instance_int16 *pd)
{
	pd->p_gain =  0;
	pd->d_gain =  0;
}


void apply_pd(pd_instance_int16 *pd, int32_t input_error, float delta_filtered, uint16_t sampling_rate)
{
	pd->output = ((pd->p_gain * input_error + pd->d_gain * delta_filtered * sampling_rate) / RESOLUTION) * PD_MAX * SCALE_FACTOR;
	if(pd->output >= PD_MAX) pd->output = PD_MAX;
	else if(pd->output <= -PD_MAX) pd->output = -PD_MAX;
}
