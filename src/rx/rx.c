
#include "rx.h"
#include "targets.h"
#include "defines.h"
#include "rx_spi.h"
#include "util.h"
#include "input.h"
#include "math.h"


extern profile_rate_t profile_rate;


typedef struct {
  aux_channel_t aux[AUX_FUNCTION_MAX];
} profile_channel_t;

#define CHANNEL_MEMBERS \
  ARRAY_MEMBER(aux, AUX_FUNCTION_MAX, uint8)


profile_channel_t profile_channel = {
    
    .aux = {
            ARMING,              //AUX_ARMING
            IDLE_UP,             //AUX_IDLE_UP
            LEVELMODE,           //AUX_LEVELMODE
            RACEMODE,            //AUX_RACEMODE
            HORIZON,             //AUX_HORIZON
            STICK_BOOST_PROFILE, //AUX_STICK_BOOST_PROFILE
#ifdef STICK_TRAVEL_CHECK        //AUX_TRAVEL_CHECK
            STICK_TRAVEL_CHECK,
#else
            AUX_CHANNEL_OFF,
#endif
            HIGH_RATES, //AUX_HIGH_RATES
            LEDS_ON,    //AUX_LEDS_ON
#ifdef BUZZER_ENABLE    //AUX_BUZZER_ENABLE
            BUZZER_ENABLE,
#else
            AUX_CHANNEL_OFF,
#endif
            TURTLE, //AUX_TURTLE

#ifdef MOTORS_TO_THROTTLE_MODE //AUX_MOTORS_TO_THROTTLE_MODE
            MOTORS_TO_THROTTLE_MODE,
#else
            AUX_CHANNEL_OFF,
#endif
            RSSI,
#ifdef FPV_ON //AUX_FPV_ON
            FPV_ON,
#else
            AUX_CHANNEL_OFF,
#endif
#ifdef FN_INVERTED //AUX_FN_INVERTED
            FN_INVERTED,
#else
            AUX_CHANNEL_OFF,
#endif
        }
    
};

extern bool frSkySpiInit(rx_spi_protocol_e spiProtocol);


void rx_init(void)
{
    frSkySpiInit(0);
}


uint8_t rx_aux_on(aux_function_t function) {
  return state.aux[profile_channel.aux[function]];
}

float rx_expo(float in, float exp) {
  if (exp > 1)
    exp = 1;
  if (exp < -1)
    exp = -1;
  float ans = in * in * in * exp + in * (1 - exp);
  limitf(&ans, 1.0);
  return ans;
}

float rx_smoothing_hz(rx_protocol_t proto) {
  return RX_SMOOTHING_HZ[proto];
}

void rx_apply_expo(void) {
  vec3_t angle_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  vec3_t acro_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };

  if (profile_rate.mode == RATE_MODE_BETAFLIGHT) {
    angle_expo = profile_rate.betaflight.expo;
    acro_expo = profile_rate.betaflight.expo;
  } else {
    angle_expo = profile_rate.silverware.angle_expo;
    acro_expo = profile_rate.silverware.acro_expo;
  }

  vec3_t expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
//  if (rx_aux_on(AUX_LEVELMODE)) {
//    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
//      expo.axis[0] = angle_expo.roll;
//      expo.axis[1] = acro_expo.pitch;
//      expo.axis[2] = angle_expo.yaw;
//    } else if (rx_aux_on(AUX_HORIZON)) {
//      expo.axis[0] = acro_expo.roll;
//      expo.axis[1] = acro_expo.pitch;
//      expo.axis[2] = angle_expo.yaw;
//    } else {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = angle_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
//    }
//  } else {
//    expo.axis[0] = acro_expo.roll;
//    expo.axis[1] = acro_expo.pitch;
//    expo.axis[2] = acro_expo.yaw;
//  }

  if (expo.roll > 0.01)
    state.rx.axis[0] = rx_expo(state.rx.axis[0], expo.roll);
  if (expo.pitch > 0.01)
    state.rx.axis[1] = rx_expo(state.rx.axis[1], expo.pitch);
  if (expo.yaw > 0.01)
    state.rx.axis[2] = rx_expo(state.rx.axis[2], expo.yaw);
}

void rx_precalc() {
  for (int i = 0; i < 3; ++i) {
#ifdef RX_SMOOTHING
    static float rx_temp[4] = {0, 0, 0, 0};
    lpf(&rx_temp[i], state.rx.axis[i], FILTERCALC(state.looptime, 1.0f / (float)rx_smoothing_hz(RX_PROTOCOL)));
    state.rx_filtered.axis[i] = rx_temp[i];
    limitf(&state.rx_filtered.axis[i], 1.0);
#else
    state.rx_filtered.axis[i] = state.rx.axis[i];
    limitf(&state.rx_filtered.axis[i], 1.0);
#endif

    if (profile_rate.sticks_deadband > 0.0f) {
      if (fabsf(state.rx_filtered.axis[i]) <= profile_rate.sticks_deadband) {
        state.rx_filtered.axis[i] = 0.0f;
      } else {
        if (state.rx_filtered.axis[i] >= 0) {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], profile_rate.sticks_deadband, 1, 0, 1);
        } else {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], -profile_rate.sticks_deadband, -1, 0, -1);
        }
      }
    }
  }
  state.rx_filtered.throttle = state.rx.throttle;
}


