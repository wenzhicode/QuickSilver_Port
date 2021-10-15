#pragma once

#include "vector.h"


// Rates
typedef enum {
  RATE_MODE_SILVERWARE,
  RATE_MODE_BETAFLIGHT,
} rate_modes_t;

typedef struct {
  vec3_t max_rate;
  vec3_t acro_expo;
  vec3_t angle_expo;
} rate_mode_silverware_t;

#define SILVERWARE_RATE_MEMBERS \
  MEMBER(max_rate, vec3_t)      \
  MEMBER(acro_expo, vec3_t)     \
  MEMBER(angle_expo, vec3_t)

typedef struct {
  vec3_t rc_rate;
  vec3_t super_rate;
  vec3_t expo;
} rate_mode_betaflight_t;

#define BETAFLIGHT_RATE_MEMBERS \
  MEMBER(rc_rate, vec3_t)       \
  MEMBER(super_rate, vec3_t)    \
  MEMBER(expo, vec3_t)

typedef struct {
  rate_modes_t mode;
  rate_mode_silverware_t silverware;
  rate_mode_betaflight_t betaflight;
  float level_max_angle;
  float low_rate_mulitplier;
  float sticks_deadband;
} profile_rate_t;

#define RATE_MEMBERS                         \
  MEMBER(mode, uint8)                        \
  MEMBER(silverware, rate_mode_silverware_t) \
  MEMBER(betaflight, rate_mode_betaflight_t) \
  MEMBER(level_max_angle, float)             \
  MEMBER(low_rate_mulitplier, float)         \
  MEMBER(sticks_deadband, float)




extern profile_rate_t profile_rate;








void input_stick_vector(float rx_input[], float maxangle);
void input_rates_calc(float rates[]);


