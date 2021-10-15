




#define BRUSHLESS_TARGET
//#define BRUSHED_TARGET


/****************************Rates & Expo settings****************************/

// *************max angle for level mode
#define LEVEL_MAX_ANGLE 65.0f

// ************* low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f

//**********************************************************************************************************************
//***********************************************NEW STUFF TO PLAY WITH*****************************************************

#define THROTTLE_D_ATTENUATION
#define TDA_BREAKPOINT 0.35f
#define TDA_PERCENT 0.70f

//**********************************************************************************************************************
//***********************************************RATES & EXPO SETTINGS**************************************************

// *************Select your preffered rate calculation format (define only one)
#define SILVERWARE_RATES
//#define BETAFLIGHT_RATES

// ******************** SILVERWARE_RATES ********************
// *************rate in deg/sec
// *************for acro mode
#define MAX_RATE 860.0    //Roll & Pitch axis
#define MAX_RATEYAW 500.0 //Yaw axis (used in acro and leveling modes)

// *************EXPO from 0.00 to 1.00 , 0 = no exp
// *************positive = less sensitive near center
#define ACRO_EXPO_ROLL 0.80
#define ACRO_EXPO_PITCH 0.80
#define ACRO_EXPO_YAW 0.60

#define ANGLE_EXPO_ROLL 0.55
#define ANGLE_EXPO_PITCH 0.0
#define ANGLE_EXPO_YAW 0.55

// ******************** BETAFLIGHT_RATES ********************
#define BF_RC_RATE_ROLL 1.30
#define BF_RC_RATE_PITCH 1.30
#define BF_RC_RATE_YAW 1.30

#define BF_SUPER_RATE_ROLL 0.70
#define BF_SUPER_RATE_PITCH 0.70
#define BF_SUPER_RATE_YAW 0.70

#define BF_EXPO_ROLL 0.40
#define BF_EXPO_PITCH 0.40
#define BF_EXPO_YAW 0.40

// *************max angle for level mode
#define LEVEL_MAX_ANGLE 65.0f

// ************* low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f

// *************transmitter stick adjustable deadband for roll/pitch/yaw
// *************.01f = 1% of stick range - comment out to disable
#define STICKS_DEADBAND .01f

//#define RX_SMOOTHING

/*****************************Receiver settings*******************************/

#define ARMING AUX_CHANNEL_0
#define IDLE_UP AUX_CHANNEL_0
#define LEVELMODE AUX_CHANNEL_1
#define RACEMODE AUX_CHANNEL_2
#define HORIZON AUX_CHANNEL_OFF
#define STICK_BOOST_PROFILE AUX_CHANNEL_4
#define HIGH_RATES AUX_CHANNEL_ON
#define LEDS_ON AUX_CHANNEL_OFF
#define TURTLE AUX_CHANNEL_3 //****************turtle mode
// *************enable buzzer functionality
// *************change channel assignment from AUX_CHANNEL_OFF to a numbered aux switch if you want switch control
// *************if no channel is assigned but buzzer is set to AUX_CHANNEL_ON - buzzer will activate on LVC and FAILSAFE.
#define BUZZER_ENABLE AUX_CHANNEL_OFF
#define MOTORS_TO_THROTTLE_MODE AUX_CHANNEL_OFF
#define RSSI AUX_CHANNEL_OFF
// *************switch for fpv / other, requires fet
// *************comment out to disable
#define FPV_ON AUX_CHANNEL_OFF


// Transmitter Type Selection
#define USE_MULTI

#define USE_SERIAL_DSMX
#define USE_SERIAL_SBUS


/*****************************Voltage settings********************************/
#define LIPO_CELL_COUNT  1

#define PID_VOLTAGE_COMPENSATION
#define LEVELMODE_PID_ATTENUATION 0.90f //used to prevent oscillations in angle modes with pid_voltage_compensation enabled due to high pids

// *************compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// *************calculate above factor automatically
#define AUTO_VDROP_FACTOR

// *************lower throttle when battery below threshold - forced landing low voltage cutoff
// *************THIS FEATURE WILL BE OFF BY DEFAULT EVEN WHEN DEFINED - USE STICK GESTURE LEFT-LEFT-LEFT TO ACTIVATE THEN DOWN-DOWN-DOWN TO SAVE AS ON
// *************Led light will blink once when LVC forced landing is turned on, blink twice when turned off, and will blink multiple times upon save command
//#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// *************voltage/cell to start warning led blinking
#define VBATTLOW 3.3

// *************voltage hysteresis in volts
#define HYST 0.10

// *************automatic voltage telemetry correction/calibration factor - change the values below if voltage telemetry is inaccurate
// *************Corrects for an offset error in the telemetry measurement (same offset across the battery voltage range)
// *************Enter values in total battery volts.  This is factor is used in all voltage related calculations - ensure your transmitter is not mucking with telemetry scale before adjusting
#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20


/*****************************Filter settings*********************************/

//#define WEAK_FILTERING
//#define STRONG_FILTERING
//#define VERY_STRONG_FILTERING
//#define ALIENWHOOP_ZERO_FILTERING
#define BETA_FILTERING

#ifdef BETA_FILTERING //*** ABOVE 100 ADJUST IN INCRIMENTS OF 20, BELOW 100 ADJUST IN INCRIMENTS OF 10, nothing coded beyond 500hz

//Select Gyro Filter Type *** Select Only One type
//#define KALMAN_GYRO
#define PT1_GYRO

//Select Gyro Filter Cut Frequency
#define GYRO_FILTER_PASS1 HZ_90
#define GYRO_FILTER_PASS2 HZ_90

//dynamic D term filter
//a pt1 filter that moves up in cut hz with a parabolic relationship to applied throttle.  The theory here is
//that propwash is most likely to occur as throttle is applied in dirty air - and propwash is most significantly
// caused by latency in the D term filtering.  Therefore, the approach is to reduce latency in the lowest frequency
//range of d term filtering which is responsible for the most phase delay as increasing throttle is applied.  Noise pass-through
//will obviously increase with this approach, but when used in combination with throttle_dterm_attenuation - that gains on D will
//also be lowered with increasing throttle thereby mitigating much of the danger from reduced filtering while allowing D term to be more effective
//at eliminating propwash.  Motor noise related to rpm is known to have a quadratic relationship with increasing throttle.  While a quadratic curve
//could have been selected for this feature, a faster moving parabolic one was selected in its place as the goal is not to follow motor noise, but
//to get the filter out of the way as fast as possible in the interest of better performance and handling through reduced D filter latency when you need it most.
#define DTERM_DYNAMIC_LPF
#define DYNAMIC_FREQ_MIN 70
#define DYNAMIC_FREQ_MAX 260

//Select fixed D Term Filter Cut Frequency *** Select Only One *** (This fixed filter is best applied near the FREQ_MAX of the dynamic D term filter)
//#define DTERM_LPF_2ND_HZ 260
#define DTERM_LPF_1ST_HZ 260

//Select Motor Filter Type  (last resort filtering stage)
//#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90

#endif

/************************* *Motor output setting******************************/
/*
About dshot: https://oscarliang.com/dshot/

DShot600 – 600,000 bits/Sec
DShot300 – 300,000 bits/Sec
DShot150 – 150,000 bits/Sec



*/

#define DSHOT600
//#define DSHOT150
//#define DSHOT300


#define MOTOR_MIN_COMMAND  2.0

//**************joelucid's yaw fix
#define YAW_FIX

//**************I-term relax.  Removes roll and pitch bounce back after flips
#define I_TERM_RELAX
//#define RELAX_FACTOR 10
//#define RELAX_FREQUENCY_HZ 20

//**************joelucid's transient windup protection.  Removes roll and pitch bounce back after flips
#define TRANSIENT_WINDUP_PROTECTION

#define INVERT_YAW_PID




/********************************other****************************************/
//enables use of stick accelerator and stick transition for d term lpf 1 & 2
#define ADVANCED_PID_CONTROLLER

#define THROTTLE_SAFETY .10f

#define IDLE_THR .001f

#ifdef LVC_LOWER_THROTTLE
#define SWITCHABLE_FEATURE_2
#endif

#ifdef INVERT_YAW_PID
#define SWITCHABLE_FEATURE_3
#endif


























