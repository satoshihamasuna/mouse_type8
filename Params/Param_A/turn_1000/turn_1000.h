/*
 * turn_1000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1000_H_
#define CPP_PARAMS_TURN_1000_H_

// 値の定義は同じディレクトリのturn_1000.cppにあります。

#include "Params/typedef_run_param.h"

//k = 200

// Feedforward profile used while executing the 1.0 m/s turn body.
// It can be tuned independently from the straight profile.
extern const t_ff_gain ff_gain_turn_1000;

extern const t_pid_gain sp_gain_turn90_1000;
extern const t_pid_gain om_gain_turn90_1000;
extern const t_turn_param_table slalom_L90_1000_table;
extern const t_turn_param_table slalom_R90_1000_table;
extern const t_param param_L90_1000;
extern const t_param param_R90_1000;

extern const t_pid_gain sp_gain_turn180_1000;
extern const t_pid_gain om_gain_turn180_1000;
extern const t_turn_param_table slalom_L180_1000_table;
extern const t_turn_param_table slalom_R180_1000_table;
extern const t_param param_L180_1000;
extern const t_param param_R180_1000;

//not adjust
extern const t_pid_gain sp_gain_turnV90_1000;
extern const t_pid_gain om_gain_turnV90_1000;
extern const t_turn_param_table slalom_LV90_1000_table;
extern const t_turn_param_table slalom_RV90_1000_table;
extern const t_param param_LV90_1000;
extern const t_param param_RV90_1000;

extern const t_pid_gain sp_gain_turnIn45_1000;
extern const t_pid_gain om_gain_turnIn45_1000;
extern const t_turn_param_table slalom_inL45_1000_table;
extern const t_turn_param_table slalom_inR45_1000_table;
extern const t_param param_inL45_1000;
extern const t_param param_inR45_1000;

//k = 300
extern const t_pid_gain sp_gain_turnOut45_1000;
extern const t_pid_gain om_gain_turnOut45_1000;
extern const t_turn_param_table slalom_outL45_1000_table;
extern const t_turn_param_table slalom_outR45_1000_table;
extern const t_param param_outL45_1000;
extern const t_param param_outR45_1000;


extern const t_pid_gain sp_gain_turnIn135_1000;
extern const t_pid_gain om_gain_turnIn135_1000;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_1000_table;
extern const t_turn_param_table slalom_inR135_1000_table;
extern const t_param param_inL135_1000;
extern const t_param param_inR135_1000;

//
extern const t_pid_gain sp_gain_turnOut135_1000;
extern const t_pid_gain om_gain_turnOut135_1000;
extern const t_turn_param_table slalom_outL135_1000_table;
extern const t_turn_param_table slalom_outR135_1000_table;
extern const t_param param_outL135_1000;
extern const t_param param_outR135_1000;

extern const t_param *const mode_1000[17];

#endif /* CPP_PARAMS_TURN_1000_H_ */
