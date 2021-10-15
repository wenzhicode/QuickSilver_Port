
#include <math.h>
#include "util.h"
#include <inttypes.h>


void hpf(float *out, float delta_in, float coeff)
{
    *out = (*out + delta_in) * coeff;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{

    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;

}
float constrainf(const float in, const float min, const float max) {
  if (in > max)
    return max;
  if (in < min)
    return min;
  return in;
}


void limitf(float *input, const float limit)
{
    if (*input > limit) *input = limit;
    if (*input < - limit) *input = - limit;
}

float rcexpo(float in, float exp)
{
    float ans;
    if (exp > 1) exp = 1;
    if (exp < -1) exp = -1;
    ans = in * in * in * exp + in * (1 - exp);
    limitf(&ans, 1.0);
    return ans;
}

void constrain(float *out, float min, float max)
{
    if (*out < min) *out = min;
    else if (*out > max) *out = max;
}

float fastsin(float x)
{
    float sin1;
//always wrap input angle to -PI..PI
    while (x < -3.14159265f)
        x += 6.28318531f;

    while (x >  3.14159265f)
        x -= 6.28318531f;


//compute sine
    if (x < 0)
        sin1 = (1.27323954f + .405284735f * x) * x;
    else
        sin1 = (1.27323954f - .405284735f * x) * x;

    return sin1;

}


float fastcos(float x)
{
    x += 1.57079632f;
    return fastsin(x);
}

float Q_rsqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y = number;
  i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y)); // 1st iteration
  y = y * (threehalfs - (x2 * y * y)); // 2nd iteration, this can be removed
                                       //	y  = y * ( threehalfs - ( x2 * y * y ) );   // 3nd iteration, this can be removed

  return y;
}

uint32_t min_uint32(uint32_t a, uint32_t b) {
  if (a < b) {
    return a;
  }
  return b;
}

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
    long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
    long int b = (long int) srcTo - (long int) srcFrom;
    return (a / b) + destFrom;
}

