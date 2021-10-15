#pragma once
#include "MM32F103.h"




typedef enum {
  PID_VOLTAGE_COMPENSATION_NONE,
  PID_VOLTAGE_COMPENSATION_ACTIVE,
} pid_voltage_compensation_t;

typedef struct {
  uint8_t lipo_cell_count;
  pid_voltage_compensation_t pid_voltage_compensation;
  float vbattlow;
  float actual_battery_voltage;
  float reported_telemetry_voltage;
} profile_voltage_t;

#define VOLTAGE_MEMBERS                   \
  MEMBER(lipo_cell_count, uint8)          \
  MEMBER(pid_voltage_compensation, uint8) \
  MEMBER(vbattlow, float)                 \
  MEMBER(actual_battery_voltage, float)   \
  MEMBER(reported_telemetry_voltage, float)

  
extern profile_voltage_t profile_voltage; 
  
  
  
  
  
void vbat_init(void);
void vbat_calc(void);
