#include "motor.h"

#include "control.h"
#include "dshot.h"
#include "config.h"
#include "defines.h"
#include "util.h"


profile_motor_t profile_motor ={
#ifdef INVERT_YAW_PID
        .invert_yaw = 1,
#else
        .invert_yaw = 0,
#endif
        .digital_idle = DIGITAL_IDLE,
        .dshot_time = DSHOT_TIME_600,

#ifdef TORQUE_BOOST
        .torque_boost = TORQUE_BOOST,
#else
        .torque_boost = 0.0,
#endif
#ifdef THROTTLE_BOOST
        .throttle_boost = THROTTLE_BOOST,
#else
        .throttle_boost = 0.0,
#endif

#ifndef DISABLE_FLIP_SEQUENCER
        .turtle_throttle_percent = 10.0f,
#else
        .turtle_throttle_percent = 0.00f,
#endif
};

#ifdef MOTORS_TO_THROTTLE
#warning "MOTORS TEST MODE"
#endif

#ifdef NOMOTORS
#warning "NO MOTORS"
float tempx[4];
#endif

#ifdef BRUSHED_TARGET
#define BRUSHED_MIX_SCALING
#endif

#ifdef BRUSHLESS_TARGET
#define BRUSHLESS_MIX_SCALING
#endif

#ifdef BRUSHLESS_MIX_SCALING
#undef BRUSHED_MIX_SCALING

#ifndef AIRMODE_STRENGTH
#define AIRMODE_STRENGTH 1.0f //  Most amount of power that can be added for Airmode
#endif

#ifndef CLIPPING_LIMIT
#define CLIPPING_LIMIT 1.0f //  Most amount of power that can be pulled before clipping
#endif

#endif

#ifdef BRUSHED_MIX_SCALING
#undef BRUSHLESS_MIX_SCALING

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#ifndef MIX_THROTTLE_REDUCTION_PERCENT
#define MIX_THROTTLE_REDUCTION_PERCENT 10
#endif

// limit reduction and increase to this amount ( 0.0 - 1.0)
// 0.0 = no action
// 0.5 = reduce up to 1/2 throttle
//1.0 = reduce all the way to zero
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5
#endif

#ifndef MIX_MOTOR_MAX
#define MIX_MOTOR_MAX 1.0f
#endif

#ifndef MIX_THROTTLE_INCREASE_MAX
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif

#endif

static float overthrottlefilt = 0;

extern int pwmdir;


#ifdef MOTOR_FILTER2_ALPHA
static float motorlpf(float in, int x) {
  static float motor_filt[4];
  lpf(&motor_filt[x], in, 1 - MOTOR_FILTER2_ALPHA);
  return motor_filt[x];
}
#endif

#ifdef MOTOR_KAL
static float motor_kalman(float in, int x) {
  //the noise in the system ( variance -  squared )
  const float Q = 0.02;
  const float R = Q / (float)MOTOR_KAL;

  //initial values for the kalman filter
  static float x_est_last[4];
  static float P_last[4];

  //do a prediction
  float x_temp_est = x_est_last[x];
  float P_temp = P_last[x] + Q;

  float K = P_temp * (1.0f / (P_temp + R));
  float x_est = x_temp_est + K * (in - x_temp_est);
  float P = (1 - K) * P_temp;

  //update our last's
  P_last[x] = P;
  x_est_last[x] = x_est;

  return x_est;
}
#endif

static float motord(float in, int x) {
  float factor = profile_motor.torque_boost;
  static float lastratexx[4][4];

  float out = (+0.125f * in + 0.250f * lastratexx[x][0] - 0.250f * lastratexx[x][2] - (0.125f) * lastratexx[x][3]) * factor;

  lastratexx[x][3] = lastratexx[x][2];
  lastratexx[x][2] = lastratexx[x][1];
  lastratexx[x][1] = lastratexx[x][0];
  lastratexx[x][0] = in;

  return in + out;
}

//********************************MIXER SCALING***********************************************************
static void motor_mixer_scale_calc(float mix[4]) {

#ifdef BRUSHLESS_MIX_SCALING
  uint8_t mix_scaling = 0;

  // only enable once really in the air
  if (flags.on_ground) {
    mix_scaling = 0;
  } else {
    mix_scaling = flags.in_air;
  }

  if (mix_scaling) {
    float mix_min = 1000.0f;
    float mix_max = -1000.0f;

    for (int i = 0; i < 4; i++) {
      if (mix[i] < mix_min)
        mix_min = mix[i];
      if (mix[i] > mix_max)
        mix_max = mix[i];

      if (mix_min < (-AIRMODE_STRENGTH))
        mix_min = (-AIRMODE_STRENGTH);
      if (mix_max > (1 + CLIPPING_LIMIT))
        mix_max = (1 + CLIPPING_LIMIT);
    }

    float mix_range = mix_max - mix_min;
    float reduce_amount = 0.0f;

    if (mix_range > 1.0f) {
      float scale = 1.0f / mix_range;

      for (int i = 0; i < 4; i++)
        mix[i] *= scale;

      mix_min *= scale;
      reduce_amount = mix_min;
    } else {
      if (mix_max > 1.0f)
        reduce_amount = mix_max - 1.0f;
      else if (mix_min < 0.0f)
        reduce_amount = mix_min;
    }

    if (reduce_amount != 0.0f) {
      for (int i = 0; i < 4; i++)
        mix[i] -= reduce_amount;
    }
  }
#endif

#ifdef BRUSHED_MIX_SCALING
  //throttle reduction
  float overthrottle = 0;
  float underthrottle = 0.001f;

  for (int i = 0; i < 4; i++) {
    if (mix[i] > overthrottle)
      overthrottle = mix[i];
    if (mix[i] < underthrottle)
      underthrottle = mix[i];
  }

  overthrottle -= MIX_MOTOR_MAX;

  if (overthrottle > (float)MIX_THROTTLE_REDUCTION_MAX)
    overthrottle = (float)MIX_THROTTLE_REDUCTION_MAX;

  if (overthrottle > overthrottlefilt)
    overthrottlefilt += 0.005f;
  else
    overthrottlefilt -= 0.01f;

  if (overthrottlefilt > (float)MIX_THROTTLE_REDUCTION_MAX)
    overthrottlefilt = (float)MIX_THROTTLE_REDUCTION_MAX;
  if (overthrottlefilt < -0.1f)
    overthrottlefilt = -0.1;

  overthrottle = overthrottlefilt;

  if (overthrottle < 0.0f)
    overthrottle = -0.0001f;

  // reduce by a percentage only, so we get an inbetween performance
  overthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);

  if (overthrottle > 0) { // exceeding max motor thrust
    float temp = overthrottle;
    for (int i = 0; i < 4; i++) {
      mix[i] -= temp;
    }
  }

  //Brushed airmode - throttle increase
  if (flags.in_air == 1) {
    float underthrottle = 0;

    for (int i = 0; i < 4; i++) {
      if (mix[i] < underthrottle)
        underthrottle = mix[i];
    }

    // limit to half throttle max reduction
    if (underthrottle < -(float)MIX_THROTTLE_INCREASE_MAX)
      underthrottle = -(float)MIX_THROTTLE_INCREASE_MAX;

    if (underthrottle < 0.0f) {
      for (int i = 0; i < 4; i++)
        mix[i] -= underthrottle;
    }
  }
#endif
}

void motor_mixer_calc(float mix[4]) {

#if defined(MOTORS_TO_THROTTLE)
  flags.motortest_override = 1;
#endif


    {
    // normal mode, we set mix according to pidoutput

#ifdef INVERTED_ENABLE
    if (pwmdir == REVERSE) {
      // inverted flight
      mix[MOTOR_FR] = state.throttle + state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // FR
      mix[MOTOR_FL] = state.throttle - state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // FL
      mix[MOTOR_BR] = state.throttle + state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // BR
      mix[MOTOR_BL] = state.throttle - state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // BL

    } else
#endif
    {
      // normal mixer
      mix[MOTOR_FR] = state.throttle + state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // FR
      mix[MOTOR_FL] = state.throttle - state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // FL
      mix[MOTOR_BR] = state.throttle + state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // BR
      mix[MOTOR_BL] = state.throttle - state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // BL
    }

    for (int i = 0; i <= 3; i++) {
#ifdef MOTOR_FILTER2_ALPHA
      mix[i] = motorlpf(mix[i], i);
#endif

#ifdef MOTOR_KAL
      mix[i] = motor_kalman(mix[i], i);
#endif

      if (profile_motor.torque_boost > 0.0f) {
        mix[i] = motord(mix[i], i);
      }
    }

    motor_mixer_scale_calc(mix);
  }
}

//********************************MOTOR OUTPUT***********************************************************
void motor_output_calc(float mix[4]) {
  state.thrsum = 0; //reset throttle sum for voltage monitoring logic in main loop

  //Begin for-loop to send motor commands
  for (int i = 0; i <= 3; i++) {

#if defined(BRUSHED_TARGET)
    if (profile.motor.digital_idle && !(rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE) || flags.motortest_override)) {
      float motor_min_value = (float)profile.motor.digital_idle * 0.01f;
      //Clip all mixer values into 0 to 1 range before remapping
      mix[i] = constrainf(mix[i], 0, 1);
      mix[i] = motor_min_value + mix[i] * (1.0f - motor_min_value);
    }
#else
    // brushless: do nothing - idle set by DSHOT
#endif

#ifndef NOMOTORS
    //normal mode
    dshotSet(i, mix[i]);
#else
    // no motors mode
    // to maintain timing or it will be optimized away
    tempx[i] = motormap(mix[i]);
#endif

    // clip mixer outputs (if not already done) before applying calculating throttle sum
    mix[i] = constrainf(mix[i], 0, 1);
    state.thrsum += mix[i];
  }

  //calculate throttle sum for voltage monitoring logic in main loop
  state.thrsum = state.thrsum / 4;
}


