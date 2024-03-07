#include "pid.h"


void set_pid_gain(pid_instance_int16* pid, float p, float i, float d)
{
	pid->p_gain = p;
	pid->i_gain = i;
	pid->d_gain = d;
}

void reset_pid(pid_instance_int16 *pid)
{
	pid->p_gain =  0;
	pid->i_gain = 0;
	pid->d_gain =  0;
	pid->error_integral = 0;
}


void apply_pid(pid_instance_int16 *pid, int16_t input_error, uint16_t sampling_rate)
{
	pid->error_integral += input_error;
	
    if(pid->error_integral > INTEGRAL_GAIN_MAX) pid->error_integral = INTEGRAL_GAIN_MAX;
	
	else if(pid->error_integral < -INTEGRAL_GAIN_MAX) pid->error_integral = -INTEGRAL_GAIN_MAX;

	pid->output = pid->p_gain * input_error +
			pid->i_gain * (pid->error_integral) / sampling_rate +
			pid->d_gain * sampling_rate * (input_error - pid->last_error);


	if(pid->output >= PID_MAX) pid->output = PID_MAX;
	
	if(pid->output <= -PID_MAX) pid->output = -PID_MAX;
	
	pid->last_error = input_error;
}
