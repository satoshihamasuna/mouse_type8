/*
 * turn_1800.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1800_H_
#define CPP_PARAMS_TURN_1800_H_

// 値の定義は同じディレクトリのturn_1800.cppにあります。

#include "Params/typedef_run_param.h"

// OM feedforward starts from the corresponding 1600 mm/s turn and is refined
// per shape from the 1800 mm/s voltage/response reports. Short profiles use
// shape-specific jerk gains to limit saturation; accel/decel gains account for
// first/second-half tracking and post-turn residual omega. Turn-specific SP
// feedforward uses the conservative 50% response-model trial value from the
// 20260722 translational analysis.  Only measured shapes receive this value.
// Recheck in45/V90 carefully because their source logs include saturation.
extern const t_ff_gain ff_gain_long_turn_90_R_1800;
extern const t_ff_gain ff_gain_long_turn_90_L_1800;
extern const t_ff_gain ff_gain_long_turn_180_R_1800;
extern const t_ff_gain ff_gain_long_turn_180_L_1800;
extern const t_ff_gain ff_gain_in45_R_1800;
extern const t_ff_gain ff_gain_in45_L_1800;
extern const t_ff_gain ff_gain_out45_R_1800;
extern const t_ff_gain ff_gain_out45_L_1800;
extern const t_ff_gain ff_gain_in135_R_1800;
extern const t_ff_gain ff_gain_in135_L_1800;
extern const t_ff_gain ff_gain_out135_R_1800;
extern const t_ff_gain ff_gain_out135_L_1800;
extern const t_ff_gain ff_gain_v90_R_1800;
extern const t_ff_gain ff_gain_v90_L_1800;


//k = 250
extern const t_pid_gain sp_gain_turn90_1800;
extern const t_pid_gain om_gain_turn90_1800;
//const static t_turn_param_table slalom_L90_1800_table = {1.80f, 52.0f,13.71,38.14, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1800_table = {1.80f,-52.0f,13.71,38.14,-90.0f,Turn_R};
// Lstart/Lend are the medians of the 20260725 Lstart=Lend=0 right-turn
// marker videos.  Apply the measured right-turn values to both directions.
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L90_1800_table;
extern const t_turn_param_table slalom_R90_1800_table;
#else
extern const t_turn_param_table slalom_L90_1800_table;
extern const t_turn_param_table slalom_R90_1800_table;
#endif
extern const t_param param_L90_1800;
extern const t_param param_R90_1800;
//k = 250
extern const t_pid_gain sp_gain_turn180_1800;
extern const t_pid_gain om_gain_turn180_1800;
//const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,9.90,39.15, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,9.90,39.15,-180.0f,Turn_R};
/*20260725_*/
//const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,22.51,32.38, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,22.51,32.38,-180.0f,Turn_R};
// 2026-11-21採用の暫定値。左右の角度補正は鏡映し、旧マーカー長切替から独立。
extern const t_turn_param_table slalom_L180_1800_table;
extern const t_turn_param_table slalom_R180_1800_table;
extern const t_param param_L180_1800;
extern const t_param param_R180_1800;
//k = 300
//not adjust
extern const t_pid_gain sp_gain_turnV90_1800;
extern const t_pid_gain om_gain_turnV90_1800;
//const static t_turn_param_table slalom_LV90_1800_table = {1.80f, 40.0f,3.10,27.38+00.0, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1800_table = {1.80f,-40.0f,3.10,27.38+00.0,-90.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 videos, R median copied to L/R; raw Lstart -1.46 mm clamped to zero).
extern const t_turn_param_table slalom_LV90_1800_table;
extern const t_turn_param_table slalom_RV90_1800_table;
#else
extern const t_turn_param_table slalom_LV90_1800_table;
extern const t_turn_param_table slalom_RV90_1800_table;
#endif
extern const t_param param_LV90_1800;
extern const t_param param_RV90_1800;
//k = 250
extern const t_pid_gain sp_gain_turnIn45_1800;
extern const t_pid_gain om_gain_turnIn45_1800;
//const static t_turn_param_table slalom_inL45_1800_table = {1.80f, 53.0f,9.27,41.09, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1800_table = {1.80f,-53.0f,9.27,41.09,-45.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_inL45_1800_table;
extern const t_turn_param_table slalom_inR45_1800_table;
#else
extern const t_turn_param_table slalom_inL45_1800_table;
extern const t_turn_param_table slalom_inR45_1800_table;
#endif
extern const t_param param_inL45_1800;
extern const t_param param_inR45_1800;

//k = 250
extern const t_pid_gain sp_gain_turnOut45_1800;
extern const t_pid_gain om_gain_turnOut45_1800;
//const static t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.81,22.01, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,23.81,22.01,-45.0f,Turn_R};
// 左右45度退出は撮影方式フラグによらず、turn_1800.cppの暫定採用値を使う。
extern const t_turn_param_table slalom_outL45_1800_table;
extern const t_turn_param_table slalom_outR45_1800_table;
extern const t_param param_outL45_1800;
extern const t_param param_outR45_1800;

//k = 250
extern const t_pid_gain sp_gain_turnIn135_1800;
extern const t_pid_gain om_gain_turnIn135_1800;//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1800_table = {1.80f, 43.0f,10.67,31.00, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1800_table = {1.80f,-43.0f,10.67,31.00,-135.0f,Turn_R};
// 左右135度進入は撮影方式フラグによらず、turn_1800.cppの単体調整値を使う。
extern const t_turn_param_table slalom_inL135_1800_table;
extern const t_turn_param_table slalom_inR135_1800_table;
extern const t_param param_inL135_1800;
extern const t_param param_inR135_1800;

//k = 250
extern const t_pid_gain sp_gain_turnOut135_1800;
extern const t_pid_gain om_gain_turnOut135_1800;
//const static t_turn_param_table slalom_outL135_1800_table = {1.80f, 41.0f,9.30,44.12, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1800_table = {1.80f,-41.0f,9.30,44.12,-135.0f,Turn_R};
// 左右135度退出は撮影方式フラグによらず、turn_1800.cppの暫定採用値を使う。
extern const t_turn_param_table slalom_outL135_1800_table;
extern const t_turn_param_table slalom_outR135_1800_table;
extern const t_param param_outL135_1800;
extern const t_param param_outR135_1800;

extern const t_param *const mode_1800[17];

extern const t_param *const mode_1800_acc_v1[19];

extern const t_param *const mode_1800_acc_v2[19];



#endif /* CPP_PARAMS_TURN_1800_H_ */
