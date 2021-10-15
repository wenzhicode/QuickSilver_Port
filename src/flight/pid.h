#pragma once

#include "vector.h"



typedef enum {
  PID_PROFILE_1,
  PID_PROFILE_2,
  PID_PROFILE_MAX
} pid_profile_t;

typedef struct {
  vec3_t accelerator;
  vec3_t transition;
} stick_rate_t;

#define STICK_RATE_MEMBERS    \
  MEMBER(accelerator, vec3_t) \
  MEMBER(transition, vec3_t)

typedef enum {
  STICK_PROFILE_OFF,
  STICK_PROFILE_ON,
  STICK_PROFILE_MAX
} stick_profile_t;

typedef enum {
  THROTTLE_D_ATTENTUATION_NONE,
  THROTTLE_D_ATTENUATION_ACTIVE,
  THROTTLE_D_ATTENUATION_MAX,
} tda_active_t;

typedef struct {
  tda_active_t tda_active;
  float tda_breakpoint;
  float tda_percent;
} throttle_dterm_attenuation_t;

#define DTERM_ATTENUATION_MEMBERS \
  MEMBER(tda_active, uint8)       \
  MEMBER(tda_breakpoint, float)   \
  MEMBER(tda_percent, float)

  
  
typedef struct {
  vec3_t kp;
  vec3_t ki;
  vec3_t kd;
} pid_rate_t;

#define PID_RATE_MEMBERS \
  MEMBER(kp, vec3_t)     \
  MEMBER(ki, vec3_t)     \
  MEMBER(kd, vec3_t)

typedef struct {
  float kp;
  float kd;
} angle_pid_rate_t;

#define ANGLE_PID_RATE_MEMBERS \
  MEMBER(kp, float)            \
  MEMBER(kd, float)

typedef struct {
  uint32_t index;
  const char *name;
  pid_rate_t rate;
} pid_rate_preset_t;

#define PID_RATE_PRESET_MEMBERS \
  MEMBER(index, uint32)         \
  STR_MEMBER(name)              \
  MEMBER(rate, pid_rate_t)


typedef struct {
  pid_profile_t pid_profile;
  pid_rate_t pid_rates[PID_PROFILE_MAX];
  stick_profile_t stick_profile;
  stick_rate_t stick_rates[STICK_PROFILE_MAX];
  angle_pid_rate_t big_angle;
  angle_pid_rate_t small_angle;
  throttle_dterm_attenuation_t throttle_dterm_attenuation;
} profile_pid_t;

#define PID_MEMBERS                                          \
  MEMBER(pid_profile, uint8)                                 \
  ARRAY_MEMBER(pid_rates, PID_PROFILE_MAX, pid_rate_t)       \
  MEMBER(stick_profile, uint8)                               \
  ARRAY_MEMBER(stick_rates, STICK_PROFILE_MAX, stick_rate_t) \
  MEMBER(big_angle, angle_pid_rate_t)                        \
  MEMBER(small_angle, angle_pid_rate_t)                      \
  MEMBER(throttle_dterm_attenuation, throttle_dterm_attenuation_t)



extern profile_pid_t profile_pid;






void rotateErrors(void);

void pid_init(void);
void pid_precalc(void);
float pid(int x);

int next_pid_term(void); // Return value : 0 - p, 1 - i, 2 - d
int next_pid_axis(void); // Return value : 0 - Roll, 1 - Pitch, 2 - Yaw
int increase_pid(void);
int decrease_pid(void);

