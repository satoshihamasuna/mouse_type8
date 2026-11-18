/*
 * turn_2000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_2000_H_
#define CPP_PARAMS_TURN_2000_H_

// 値の定義は同じディレクトリのturn_2000.cppにあります。

#include "Params/typedef_run_param.h"

// OM feedforward tuned from the 2026-07-21 2000 mm/s turn logs.
// R/L-specific SP turn feedforward was identified independently from the
// 20260722 long-180 voltage response.  Long-180 uses a second 50% iterative
// correction from the repeated FF-on logs.  Other shapes retain the initial
// directional baseline because spreading the stronger correction would drive
// measured long-90 profiles into the common 1.0 V clamp.
extern const t_ff_gain ff_gain_long_turn_90_R_2000;
extern const t_ff_gain ff_gain_long_turn_90_L_2000;
extern const t_ff_gain ff_gain_long_turn_180_R_2000;
extern const t_ff_gain ff_gain_long_turn_180_L_2000;
extern const t_ff_gain ff_gain_in45_R_2000;
extern const t_ff_gain ff_gain_in45_L_2000;
extern const t_ff_gain ff_gain_out45_R_2000;
extern const t_ff_gain ff_gain_out45_L_2000;
extern const t_ff_gain ff_gain_in135_R_2000;
extern const t_ff_gain ff_gain_in135_L_2000;
extern const t_ff_gain ff_gain_out135_R_2000;
extern const t_ff_gain ff_gain_out135_L_2000;
extern const t_ff_gain ff_gain_v90_R_2000;
extern const t_ff_gain ff_gain_v90_L_2000;


extern const t_pid_gain sp_gain_turn90_2000;
extern const t_pid_gain om_gain_turn90_2000;
//const static t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
// Lstart/Lend are direction-specific medians from the 20260725
// Lstart=Lend=0 marker videos.
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 direction-specific Lstart=Lend=0 videos).
extern const t_turn_param_table slalom_L90_2000_table;
extern const t_turn_param_table slalom_R90_2000_table;
#else
extern const t_turn_param_table slalom_L90_2000_table;
extern const t_turn_param_table slalom_R90_2000_table;
#endif
extern const t_param param_L90_2000;
extern const t_param param_R90_2000;

extern const t_pid_gain sp_gain_turn180_2000;
extern const t_pid_gain om_gain_turn180_2000;
//const static t_turn_param_table slalom_L180_2000_table = {2.00f, 50.0f,5.28,41.08, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_2000_table = {2.00f,-50.0f,5.28,41.08,-180.0f,Turn_R};
/*20260725*/
//const static t_turn_param_table slalom_L180_2000_table = {2.00f, 52.0f,26.07,35.18, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_2000_table = {2.00f,-52.0f,28.25,38.74,-180.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (L: 20260725 direction-specific videos; R: 20260726 videos).
extern const t_turn_param_table slalom_L180_2000_table;
extern const t_turn_param_table slalom_R180_2000_table;
#else
extern const t_turn_param_table slalom_L180_2000_table;
extern const t_turn_param_table slalom_R180_2000_table;
#endif
extern const t_param param_L180_2000;
extern const t_param param_R180_2000;

//k = 300
//not adjust
extern const t_pid_gain sp_gain_turnV90_2000;
extern const t_pid_gain om_gain_turnV90_2000;
extern const t_turn_param_table slalom_LV90_2000_table;
extern const t_turn_param_table slalom_RV90_2000_table;
extern const t_param param_LV90_2000;
extern const t_param param_RV90_2000;
//k = 300
extern const t_pid_gain sp_gain_turnIn45_2000;
extern const t_pid_gain om_gain_turnIn45_2000;
extern const t_turn_param_table slalom_inL45_2000_table;
extern const t_turn_param_table slalom_inR45_2000_table;
extern const t_param param_inL45_2000;
extern const t_param param_inR45_2000;

//k = 300
extern const t_pid_gain sp_gain_turnOut45_2000;
extern const t_pid_gain om_gain_turnOut45_2000;
extern const t_turn_param_table slalom_outL45_2000_table;
extern const t_turn_param_table slalom_outR45_2000_table;
extern const t_param param_outL45_2000;
extern const t_param param_outR45_2000;


extern const t_pid_gain sp_gain_turnIn135_2000;
extern const t_pid_gain om_gain_turnIn135_2000;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_2000_table;
extern const t_turn_param_table slalom_inR135_2000_table;
extern const t_param param_inL135_2000;
extern const t_param param_inR135_2000;

//
extern const t_pid_gain sp_gain_turnOut135_2000;
extern const t_pid_gain om_gain_turnOut135_2000;
extern const t_turn_param_table slalom_outL135_2000_table;
extern const t_turn_param_table slalom_outR135_2000_table;
extern const t_param param_outL135_2000;
extern const t_param param_outR135_2000;

extern const t_param *const mode_2000[17];

extern const t_param *const mode_2000_acc[19];
extern const t_param *const mode_2000_acc_v2[17];




#endif /* CPP_PARAMS_TURN_2000_H_ */
