#pragma once

#include "MM32F103.h"



typedef enum {
  DSHOT_TIME_150 = 150,
  DSHOT_TIME_300 = 300,
  DSHOT_TIME_600 = 600,
} dshot_time_t;

typedef struct {
  float digital_idle;
  dshot_time_t dshot_time;
  uint8_t invert_yaw;
  uint8_t gyro_orientation;
  float torque_boost;
  float throttle_boost;
  float turtle_throttle_percent;
} profile_motor_t;

#define MOTOR_MEMBERS                \
  MEMBER(digital_idle, float)        \
  MEMBER(dshot_time, uint16)         \
  MEMBER(invert_yaw, uint8)          \
  MEMBER(gyro_orientation, uint8)    \
  MEMBER(torque_boost, float)        \
  MEMBER(throttle_boost, float)      \
  ARRAY_MEMBER(motor_pins, 4, uint8) \
  MEMBER(turtle_throttle_percent, float)


extern profile_motor_t profile_motor;











void motor_mixer_calc(float mix[4]);
void motor_output_calc(float mix[4]);




