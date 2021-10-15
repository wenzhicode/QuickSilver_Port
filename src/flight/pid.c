#include "pid.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "config.h"
#include "control.h"
#include "filter.h"
#include "util.h"
#include "defines.h"
#include "mpu6000.h"
#include "vbat.h"

profile_pid_t profile_pid = {
    
    .pid_profile = PID_PROFILE_1,
        .pid_rates = {
            {
                .kp = {127, 127, 148},
                .ki = {77, 77, 77},
                .kd = {101, 101, 13},
            },
            {
                .kp = {127, 127, 148},
                .ki = {77, 77, 77},
                .kd = {101, 101, 13},
            }
        },
        .stick_profile = STICK_PROFILE_OFF,
        .stick_rates = {
            //**************************ADVANCED PID CONTROLLER - WITH PROFILE SWITCHING ON AUX SWITCH STICK_BOOST_PROFILE*******************************
            // GENERAL SUMMARY OF THIS FEATURE:
            // stickAccelerator and stickTransition are a more detailed version of the traditional D term setpoint weight and transition variables that you may be familiar with in other firmwares.
            // The difference here is that we name the D term setpoint weight "Stick Accelerator" because it's actual function is to accelerate the response of the pid controller to stick inputs.
            // Another difference is that negative stick transitions are possible meaning that you can have a higher stick acceleration near center stick which fades to a lower stick acceleration at
            // full stick throws should you desire to see what that feels like.  Traditionally we are only used to being able to transition from a low setpoint to a higher one.
            // The final differences are that you can adjust each axis independently and also set up two seperate profiles so that you can switch "feels" in flight with the STICK_BOOST_PROFILE aux
            // channel selection set up in the receiver section of config.h
            //
            //HOW TO USE THIS FEATURE:
            // Safe values for stickAccelerator are from 0 to about 2.5 where 0 represents a "MEASUREMENT" based D term calculation and is the traditional Silverware PID controller, and a
            // a value of 1 represents an "ERROR" based D term calculation.  Values above 1 add even more acceleration but be reasonable and keep this below about 2.5.

            // Range of acceptable values for stickTransition are from -1 to 1.  Do not input a value outside of this range.  When stick transition is 0 - no stick transition will take place
            // and stick acceleration will remain constant regardless of stick position.  Positive values up to 1 will represent a transition where stick acceleration at it's maximum at full
            // stick deflection and is reduced by whatever percentage you enter here at stick center.  For example accelerator at 1 and transition at .3 means that there will be 30% reduction
            // of acceleration at stick center, and acceleration strength of 1 at full stick.
            {
                //pid profile A	Roll  PITCH  YAW
                .accelerator = {0.0, 0.0, 0.0}, //keep values between 0 and 2.5
                .transition = {0.0, 0.0, 0.0},  //keep values between -1 and 1
            },
            {
                //pid profile B	Roll  PITCH  YAW
                .accelerator = {1.5, 1.5, 1.0}, //keep values between 0 and 2.5
                .transition = {0.3, 0.3, 0.0},  //keep values between -1 and 1
            },
        },
        //**************************** ANGLE PIDS - used in level mode to set leveling strength

        // Leveling algorithm coefficients for small errors  (normal flying)
        .small_angle = {
            .kp = 10.00, // P TERM GAIN ROLL + PITCH
            .kd = 3.0,   // D TERM GAIN ROLL + PITCH
        },

        // Leveling algorithm coefficients for large errors  (stick banging or collisions)
        .big_angle = {
            .kp = 5.00, // P TERM GAIN ROLL + PITCH
            .kd = 0.0,  // D TERM GAIN ROLL + PITCH
        },

        .throttle_dterm_attenuation = {
#ifdef THROTTLE_D_ATTENUATION
            .tda_active = THROTTLE_D_ATTENUATION_ACTIVE,
#else
            .tda_active = THROTTLE_D_ATTENUATION_NONE,
#endif
            .tda_breakpoint = TDA_BREAKPOINT,
            .tda_percent = TDA_PERCENT,
        },
 
};





//************************************Setpoint Weight****************************************
#ifdef BRUSHLESS_TARGET

/// output limit
const float outlimit[PID_SIZE] = {0.8, 0.8, 0.4};

// limit of integral term (abs)
const float integrallimit[PID_SIZE] = {0.8, 0.8, 0.4};

#else //BRUSHED TARGET

// "p term setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
#define ENABLE_SETPOINT_WEIGHTING
//            Roll   Pitch   Yaw
//float b[3] = { 0.97 , 0.98 , 0.95};   //BRUSHED RACE
float b[3] = {0.93, 0.93, 0.9}; //BRUSHED FREESTYLE

/// output limit
const float outlimit[PID_SIZE] = {1.7, 1.7, 0.5};

// limit of integral term (abs)
const float integrallimit[PID_SIZE] = {1.7, 1.7, 0.5};

#endif

// TODO: re-implement?
//#define ANTI_WINDUP_DISABLE





int number_of_increments[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
int current_pid_axis = 0;
int current_pid_term = 0;

static float lasterror[PID_SIZE];
static float lasterror2[PID_SIZE];
 float current_kp[PID_SIZE] = {0, 0, 0};
 float current_ki[PID_SIZE] = {0, 0, 0};
 float current_kd[PID_SIZE] = {0, 0, 0};

static float ierror[PID_SIZE] = {0, 0, 0};

static float v_compensation = 1.00;
static float tda_compensation = 1.00;

// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f from H101 code
#define PID_VC_FACTOR 1.33f
float timefactor;

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static filter_lp_pt1 dynamic_filter;
static filter_state_t dynamic_filter_state[3];

void pid_init() {

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile_filter.dterm[i].type, &filter[i], filter_state[i], 3, profile_filter.dterm[i].cutoff_freq);
  }

  if (profile_filter.dterm_dynamic_enable) {
    // zero out filter, freq will be updated later on
    filter_lp_pt1_init(&dynamic_filter, dynamic_filter_state, 3, DYNAMIC_FREQ_MAX);
  }
}


pid_rate_t *profile_current_pid_rates() {
  return &profile_pid.pid_rates[profile_pid.pid_profile];
}


// calculate change from ideal loop time
// 0.0032f is there for legacy purposes, should be 0.001f = looptime
// this is called in advance as an optimization because it has division
void pid_precalc() {
  timefactor = 0.0032f / state.looptime;

  filter_coeff(profile_filter.dterm[0].type, &filter[0], profile_filter.dterm[0].cutoff_freq);
  filter_coeff(profile_filter.dterm[1].type, &filter[1], profile_filter.dterm[1].cutoff_freq);

  if (profile_voltage.pid_voltage_compensation) {
    v_compensation = mapf((state.vbattfilt_corr / (float)state.lipo_cell_count), 2.5f, 3.85f, PID_VC_FACTOR, 1.0f);
    v_compensation = constrainf(v_compensation, 1.0f, PID_VC_FACTOR);

#ifdef LEVELMODE_PID_ATTENUATION
    if (rx_aux_on(AUX_LEVELMODE))
      v_compensation *= LEVELMODE_PID_ATTENUATION;
#endif
  }

  if (profile_pid.throttle_dterm_attenuation.tda_active) {
    tda_compensation = mapf(state.throttle, profile_pid.throttle_dterm_attenuation.tda_breakpoint, 1.0f, 1.0f, profile_pid.throttle_dterm_attenuation.tda_percent);
    tda_compensation = constrainf(tda_compensation, profile_pid.throttle_dterm_attenuation.tda_percent, 1.0f);
  }

  if (profile_filter.dterm_dynamic_enable) {
    float dynamic_throttle = state.throttle * (1 - state.throttle / 2.0f) * 2.0f;
    float d_term_dynamic_freq = mapf(dynamic_throttle, 0.0f, 1.0f, profile_filter.dterm_dynamic_min, profile_filter.dterm_dynamic_max);
    d_term_dynamic_freq = constrainf(d_term_dynamic_freq, profile_filter.dterm_dynamic_min, profile_filter.dterm_dynamic_max);

    filter_lp_pt1_coeff(&dynamic_filter, d_term_dynamic_freq);
  }

  for (uint8_t i = 0; i < PID_SIZE; i++) {
    current_kp[i] = profile_current_pid_rates()->kp.axis[i] / pid_scales[0][i];
    current_ki[i] = profile_current_pid_rates()->ki.axis[i] / pid_scales[1][i];
    current_kd[i] = profile_current_pid_rates()->kd.axis[i] / pid_scales[2][i];
  }
}

// pid calculation for acro ( rate ) mode
// input: error[x] = setpoint - gyro
// output: state.pidoutput.axis[x] = change required from motors
float pid(int x) {

  // in level mode or horizon but not racemode and while on the ground...
  if ((rx_aux_on(AUX_LEVELMODE)) && (!rx_aux_on(AUX_RACEMODE)) && ((flags.on_ground) || (flags.in_air == 0))) {
    // wind down the integral error
    ierror[x] *= 0.98f;
  } else if (flags.on_ground) {
    //in acro mode - only wind down integral when idle up is off and throttle is 0
    ierror[x] *= 0.98f;
  }

  int iwindup = 0; // (iwidup = 0  windup is permitted)   (iwindup = 1 windup is squashed)
  if ((state.pidoutput.axis[x] >= outlimit[x]) && (state.error.axis[x] > 0)) {
    iwindup = 1;
  }

  if ((state.pidoutput.axis[x] == -outlimit[x]) && (state.error.axis[x] < 0)) {
    iwindup = 1;
  }

#ifdef I_TERM_RELAX //  Roll - Pitch  Setpoint based I term relax method
#ifndef RELAX_FACTOR
#define RELAX_FACTOR 10 //  5.7 degrees/s
#endif
#ifndef RELAX_FREQUENCY
#define RELAX_FREQUENCY_HZ 20
#endif
  static float avgSetpoint[2];
  if (x < 2) {
    lpf(&avgSetpoint[x], state.setpoint.axis[x], FILTERCALC(state.looptime, 1.0f / (float)RELAX_FREQUENCY_HZ)); // 20 Hz filter
    const float hpfSetpoint = state.setpoint.axis[x] - avgSetpoint[x];
    if (fabsf(hpfSetpoint) > (float)RELAX_FACTOR * 1e-2f) {
      iwindup = 1;
    }
  }
#endif

  //SIMPSON_RULE_INTEGRAL
  if (!iwindup) {
    // assuming similar time intervals
    ierror[x] = ierror[x] + 0.166666f * (lasterror2[x] + 4 * lasterror[x] + state.error.axis[x]) * current_ki[x] * state.looptime;
    lasterror2[x] = lasterror[x];
    lasterror[x] = state.error.axis[x];
  }
  limitf(&ierror[x], integrallimit[x]);

#ifdef ENABLE_SETPOINT_WEIGHTING
  // P term
  state.pid_p_term.axis[x] = state.error.axis[x] * (b[x]) * current_kp[x];
  // b
  state.pid_p_term.axis[x] += -(1.0f - b[x]) * current_kp[x] * state.gyro.axis[x];
#else
  // P term with b disabled
  state.pid_p_term.axis[x] = state.error.axis[x] * current_kp[x];
#endif

  // Pid Voltage Comp applied to P term only
  if (profile_voltage.pid_voltage_compensation)
    state.pid_p_term.axis[x] *= v_compensation;

  // I term
  state.pid_i_term.axis[x] = ierror[x];

  // D term
  // skip yaw D term if not set
  if (current_kd[x] > 0) {
    float transitionSetpointWeight[3];
    float stickAccelerator[3];
    float stickTransition[3];
    if (rx_aux_on(AUX_STICK_BOOST_PROFILE)) {
      stickAccelerator[x] = profile_pid.stick_rates[STICK_PROFILE_ON].accelerator.axis[x];
      stickTransition[x] = profile_pid.stick_rates[STICK_PROFILE_ON].transition.axis[x];
    } else {
      stickAccelerator[x] = profile_pid.stick_rates[STICK_PROFILE_OFF].accelerator.axis[x];
      stickTransition[x] = profile_pid.stick_rates[STICK_PROFILE_OFF].transition.axis[x];
    }
    if (stickAccelerator[x] < 1) {
      transitionSetpointWeight[x] = (fabsf(state.rx_filtered.axis[x]) * stickTransition[x]) + (1 - stickTransition[x]);
    } else {
      transitionSetpointWeight[x] = (fabsf(state.rx_filtered.axis[x]) * (stickTransition[x] / stickAccelerator[x])) + (1 - stickTransition[x]);
    }

    static float lastrate[3];
    static float lastsetpoint[3];
    static float setpoint_derivative[3];

#ifdef RX_SMOOTHING
    lpf(&setpoint_derivative[x], ((state.setpoint.axis[x] - lastsetpoint[x]) * current_kd[x] * timefactor), FILTERCALC(state.looptime, 1.0f / (float)rx_smoothing_hz(RX_PROTOCOL)));
#else
    setpoint_derivative[x] = (state.setpoint.axis[x] - lastsetpoint[x]) * current_kd[x] * timefactor;
#endif

    float gyro_derivative = (state.gyro.axis[x] - lastrate[x]) * current_kd[x] * timefactor;
    if (profile_pid.throttle_dterm_attenuation.tda_active)
      gyro_derivative *= tda_compensation;

    const float dterm = (setpoint_derivative[x] * stickAccelerator[x] * transitionSetpointWeight[x]) - (gyro_derivative);
    lastsetpoint[x] = state.setpoint.axis[x];
    lastrate[x] = state.gyro.axis[x];

    //D term filtering
    float dlpf = dterm;

    dlpf = filter_step(profile_filter.dterm[0].type, &filter[0], &filter_state[0][x], dlpf);
    dlpf = filter_step(profile_filter.dterm[1].type, &filter[1], &filter_state[1][x], dlpf);

    if (profile_filter.dterm_dynamic_enable) {
      dlpf = filter_lp_pt1_step(&dynamic_filter, &dynamic_filter_state[x], dlpf);
    }

    state.pid_d_term.axis[x] = dlpf;
  }

  state.pidoutput.axis[x] = state.pid_p_term.axis[x] + state.pid_i_term.axis[x] + state.pid_d_term.axis[x];
  limitf(&state.pidoutput.axis[x], outlimit[x]);

  return state.pidoutput.axis[x];
}


void rotateErrors() {
#ifdef YAW_FIX
  // rotation around x axis:
  ierror[1] -= ierror[2] * state.gyro.axis[0] * state.looptime;
  ierror[2] += ierror[1] * state.gyro.axis[0] * state.looptime;

  // rotation around y axis:
  ierror[2] -= ierror[0] * state.gyro.axis[1] * state.looptime;
  ierror[0] += ierror[2] * state.gyro.axis[1] * state.looptime;

  // rotation around z axis:
  ierror[0] -= ierror[1] * state.gyro.axis[2] * state.looptime;
  ierror[1] += ierror[0] * state.gyro.axis[2] * state.looptime;
#endif
}


