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
//k = 250
extern const t_pid_gain sp_gain_turn90_1800;;
extern const t_pid_gain om_gain_turn90_1800;
extern const t_turn_param_table slalom_L90_1800_table;
extern const t_turn_param_table slalom_R90_1800_table;
extern const t_param param_L90_1800;
extern const t_param param_R90_1800;
//k = 250
extern const t_pid_gain sp_gain_turn180_1800;;
extern const t_pid_gain om_gain_turn180_1800;
extern const t_turn_param_table slalom_L180_1800_table;
extern const t_turn_param_table slalom_R180_1800_table;
extern const t_param param_L180_1800;
extern const t_param param_R180_1800;
//k = 300
//not adjust
extern const t_pid_gain sp_gain_turnV90_1800;;
extern const t_pid_gain om_gain_turnV90_1800;
extern const t_turn_param_table slalom_LV90_1800_table;
extern const t_turn_param_table slalom_RV90_1800_table;
extern const t_param param_LV90_1800;
extern const t_param param_RV90_1800;
//k = 250
extern const t_pid_gain sp_gain_turnIn45_1800;;
extern const t_pid_gain om_gain_turnIn45_1800;
extern const t_turn_param_table slalom_inL45_1800_table;
extern const t_turn_param_table slalom_inR45_1800_table;
extern const t_param param_inL45_1800;
extern const t_param param_inR45_1800;

//k = 250
extern const t_pid_gain sp_gain_turnOut45_1800;;
extern const t_pid_gain om_gain_turnOut45_1800;
extern const t_turn_param_table slalom_outL45_1800_table;
extern const t_turn_param_table slalom_outR45_1800_table;
extern const t_param param_outL45_1800;
extern const t_param param_outR45_1800;

//k = 250
extern const t_pid_gain sp_gain_turnIn135_1800;;
extern const t_pid_gain om_gain_turnIn135_1800;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_1800_table;
extern const t_turn_param_table slalom_inR135_1800_table;
extern const t_param param_inL135_1800;
extern const t_param param_inR135_1800;

//k = 250
extern const t_pid_gain sp_gain_turnOut135_1800;;
extern const t_pid_gain om_gain_turnOut135_1800;
extern const t_turn_param_table slalom_outL135_1800_table;
extern const t_turn_param_table slalom_outR135_1800_table;
extern const t_param param_outL135_1800;
extern const t_param param_outR135_1800;

extern const t_param *const mode_1800[17];

extern const t_param *const mode_1800_acc_v1[19];

extern const t_param *const mode_1800_acc_v2[19];



#endif /* CPP_PARAMS_TURN_1800_H_ */
