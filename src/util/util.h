#include "MM32F103.h"

void constrain(float *out, float min, float max);
float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void lpf(float *out, float in, float coeff);
void hpf(float *out, float in, float coeff);
float rcexpo(float x, float exp);

void limitf(float *input, const float limit);

float fastsin(float x);
float fastcos(float x);


void limit180(float *);


float Q_rsqrt(float number);

uint32_t min_uint32(uint32_t a, uint32_t b);
int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo);

float constrainf(const float in, const float min, const float max) ;



