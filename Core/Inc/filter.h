#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"


#define MOVING_AVERAGE_LENGTH  30
#define EMA_LENGTH 30
#define RN_VALUES_10 {1.0, 0.6411803884299546, 0.41111229050718745, 0.26359713811572677, 0.1690133154060661, 0.10836802322189586, 0.06948345122280154, 0.04455142624448969, 0.02856550078455038, 0.01831563888873418}
#define RN_VALUES_30 { \
    1.0, \
    0.8711587695892689, \
    0.7589176018322888, \
    0.6611377242318554, \
    0.5759559263708725, \
    0.5017490561548967, \
    0.43710309040247686, \
    0.38078619041868866, \
    0.33172522912172986, \
    0.28898534244340446, \
    0.2517521153523297, \
    0.21931606305183127, \
    0.19105911163939585, \
    0.1664428206145948, \
    0.1449981228135778, \
    0.12631638626303013, \
    0.11004162763586417, \
    0.09586372893485993, \
    0.08351252814713174, \
    0.07275267126594448, \
    0.06337912758437274, \
    0.055213282804043445, \
    0.04809953551255484, \
    0.0419023321749326, \
    0.03650358414043512, \
    0.031800417445379814, \
    0.02770321253414219, \
    0.024133896544913325, \
    0.021024455619461393, \
    0.01831563888873418 \
}




typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int32_t sum;
	float out;
} moving_avg_obj;

typedef struct{
	float buffer[EMA_LENGTH];
	float sum;
	float out;
} ema_obj;


void reset_average_filter(moving_avg_obj* instance);
void apply_average_filter(moving_avg_obj* instance, int16_t input, float* out);
void apply_average_filter_unsigned(moving_avg_obj* instance, uint16_t input, uint16_t* out);

void apply_ema_filter(ema_obj* instance, uint16_t input, float* out);
void reset_ema_filter(ema_obj* instance);



#endif /* INC_MOVING_AVERAGE_INT16_H_ */