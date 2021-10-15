#include <stdbool.h>

#include "control.h"
#include "math.h"
#include "pid.h"
#include "util.h"
#include "defines.h"


// code variables below
#define OUTLIMIT_FLOAT (profile_pid.small_angle.kp + profile_pid.big_angle.kp) //set angle pid output limit to sum of both P terms just in case

static float apidoutput1[ANGLE_PID_SIZE];
static float apidoutput2[ANGLE_PID_SIZE];
static float apidoutput[ANGLE_PID_SIZE];
static float lasterror[ANGLE_PID_SIZE];

float angle_pid(int x) {
  // P term 1 weighted
  apidoutput1[x] = (1 - fabsf(state.angleerror[x])) * state.angleerror[x] * profile_pid.small_angle.kp;

  // P term 2 weighted
  apidoutput2[x] = fabsf(state.angleerror[x]) * state.angleerror[x] * profile_pid.big_angle.kp;

  extern float timefactor;
  // D term 1 weighted + P term 1 weighted
  apidoutput1[x] += (state.angleerror[x] - lasterror[x]) * profile_pid.small_angle.kd * (1 - fabsf(state.angleerror[x])) * timefactor;

  // D term 2 weighted + P term 2 weighted
  apidoutput2[x] += ((state.angleerror[x] - lasterror[x]) * profile_pid.big_angle.kd * fabsf(state.angleerror[x]) * timefactor);

  // apidoutput sum
  apidoutput[x] = apidoutput1[x] + apidoutput2[x];

  lasterror[x] = state.angleerror[x];
  limitf(&apidoutput[x], OUTLIMIT_FLOAT);

  return apidoutput[x];
}


