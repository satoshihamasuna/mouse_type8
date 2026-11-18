/*
 * turn_1400.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1400_H_
#define CPP_PARAMS_TURN_1400_H_

// 値の定義は同じディレクトリのturn_1400.cppにあります。

#include "Params/typedef_run_param.h"

// Long-90 OM feedforward identified from the 20260719 fixed-PID turn-body logs.
extern const t_ff_gain ff_gain_long_turn_90_R_1400;
extern const t_ff_gain ff_gain_long_turn_90_L_1400;
extern const t_ff_gain ff_gain_long_turn_180_R_1400;
extern const t_ff_gain ff_gain_long_turn_180_L_1400;
extern const t_ff_gain ff_gain_in45_R_1400;
extern const t_ff_gain ff_gain_in45_L_1400;
extern const t_ff_gain ff_gain_out45_R_1400;
extern const t_ff_gain ff_gain_out45_L_1400;
extern const t_ff_gain ff_gain_in135_R_1400;
extern const t_ff_gain ff_gain_in135_L_1400;
extern const t_ff_gain ff_gain_out135_R_1400;
extern const t_ff_gain ff_gain_out135_L_1400;
extern const t_ff_gain ff_gain_v90_R_1400;
extern const t_ff_gain ff_gain_v90_L_1400;

//k = 250
extern const t_pid_gain sp_gain_turn90_1400;
extern const t_pid_gain om_gain_turn90_1400;
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,22.01,38.31, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,22.01,38.31,-90.0f,Turn_R};
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,18.88,34.03, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,18.88,34.03,-90.0f,Turn_R};
// Lstart/Lend are the medians of the 20260725 four-times-slow-motion
// Lstart=Lend=0 right-turn marker videos.  Apply the measured right-turn
// values to both directions.
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L90_1400_table;
extern const t_turn_param_table slalom_R90_1400_table;
#else
extern const t_turn_param_table slalom_L90_1400_table;
extern const t_turn_param_table slalom_R90_1400_table;
#endif
extern const t_param param_L90_1400;
extern const t_param param_R90_1400;

//k = 250
extern const t_pid_gain sp_gain_turn180_1400;
extern const t_pid_gain om_gain_turn180_1400;
//const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.0f,12.05,28.43, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.0f,12.05,28.43,-180.0f,Turn_R};
/* 20260725_ */
//const static t_turn_param_table slalom_L180_1400_table = {1.40f, 45.0f,24.62,29.51, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1400_table = {1.40f,-45.0f,24.62,29.51,-180.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L180_1400_table;
extern const t_turn_param_table slalom_R180_1400_table;
#else
extern const t_turn_param_table slalom_L180_1400_table;
extern const t_turn_param_table slalom_R180_1400_table;
#endif
extern const t_param param_L180_1400;
extern const t_param param_R180_1400;

//not adjust
//k = 250
extern const t_pid_gain sp_gain_turnV90_1400;
extern const t_pid_gain om_gain_turnV90_1400;
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,11.55-2.0,26.99, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,11.55-2.0,26.99,-90.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_LV90_1400_table;
extern const t_turn_param_table slalom_RV90_1400_table;
#else
extern const t_turn_param_table slalom_LV90_1400_table;
extern const t_turn_param_table slalom_RV90_1400_table;
#endif
extern const t_param param_LV90_1400;
extern const t_param param_RV90_1400;

//k = 250
extern const t_pid_gain sp_gain_turnIn45_1400;
extern const t_pid_gain om_gain_turnIn45_1400;
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,8.28,40.77, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.28,40.77,-45.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_inL45_1400_table;
extern const t_turn_param_table slalom_inR45_1400_table;
#else
extern const t_turn_param_table slalom_inL45_1400_table;
extern const t_turn_param_table slalom_inR45_1400_table;
#endif
extern const t_param param_inL45_1400;
extern const t_param param_inR45_1400;

//k = 220
//k = 250
extern const t_pid_gain sp_gain_turnOut45_1400;
extern const t_pid_gain om_gain_turnOut45_1400;
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,26.22,22.11, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,26.22,22.11,-45.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL45_1400_table;
extern const t_turn_param_table slalom_outR45_1400_table;
#else
extern const t_turn_param_table slalom_outL45_1400_table;
extern const t_turn_param_table slalom_outR45_1400_table;
#endif
extern const t_param param_outL45_1400;
extern const t_param param_outR45_1400;

//k= 250
extern const t_pid_gain sp_gain_turnIn135_1400;
extern const t_pid_gain om_gain_turnIn135_1400;//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,21.48,31.96, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,21.48,31.86,-135.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 videos, R median copied to L/R; raw Lstart -0.50 mm clamped to zero).
extern const t_turn_param_table slalom_inL135_1400_table;
extern const t_turn_param_table slalom_inR135_1400_table;
#else
extern const t_turn_param_table slalom_inL135_1400_table;
extern const t_turn_param_table slalom_inR135_1400_table;
#endif
extern const t_param param_inL135_1400;
extern const t_param param_inR135_1400;

//220
extern const t_pid_gain sp_gain_turnOut135_1400;
extern const t_pid_gain om_gain_turnOut135_1400;
//const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 42.0f,13.88,39.77, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-42.0f,13.88,39.77,-135.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL135_1400_table;
extern const t_turn_param_table slalom_outR135_1400_table;
#else
extern const t_turn_param_table slalom_outL135_1400_table;
extern const t_turn_param_table slalom_outR135_1400_table;
#endif
extern const t_param param_outL135_1400;
extern const t_param param_outR135_1400;

extern const t_param *const mode_1400[17];

extern const t_param *const mode_1400_acc[19];
#endif /* CPP_PARAMS_TURN_1400_H_ */
