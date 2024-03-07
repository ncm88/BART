#ifndef ENCODER_H
#define ENCODER_H

#include "stdint.h"
#include "tim.h"


typedef struct{
    int16_t velocity;
    int64_t position;
    uint32_t prevPos;
} encoder_instance;


void update_encoder(encoder_instance *handle, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *handle);

#endif




