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

extern const t_pid_gain sp_gain_turn90_2000;;
extern const t_pid_gain om_gain_turn90_2000;
extern const t_turn_param_table slalom_L90_2000_table;
extern const t_turn_param_table slalom_R90_2000_table;
extern const t_param param_L90_2000;
extern const t_param param_R90_2000;

extern const t_pid_gain sp_gain_turn180_2000;;
extern const t_pid_gain om_gain_turn180_2000;
extern const t_turn_param_table slalom_L180_2000_table;
extern const t_turn_param_table slalom_R180_2000_table;
extern const t_param param_L180_2000;
extern const t_param param_R180_2000;



//k = 300
//not adjust
extern const t_pid_gain sp_gain_turnV90_2000;;
extern const t_pid_gain om_gain_turnV90_2000;
extern const t_turn_param_table slalom_LV90_2000_table;
extern const t_turn_param_table slalom_RV90_2000_table;
extern const t_param param_LV90_2000;
extern const t_param param_RV90_2000;
//k = 300
extern const t_pid_gain sp_gain_turnIn45_2000;;
extern const t_pid_gain om_gain_turnIn45_2000;
extern const t_turn_param_table slalom_inL45_2000_table;
extern const t_turn_param_table slalom_inR45_2000_table;
extern const t_param param_inL45_2000;
extern const t_param param_inR45_2000;

//k = 300
extern const t_pid_gain sp_gain_turnOut45_2000;;
extern const t_pid_gain om_gain_turnOut45_2000;
extern const t_turn_param_table slalom_outL45_2000_table;
extern const t_turn_param_table slalom_outR45_2000_table;
extern const t_param param_outL45_2000;
extern const t_param param_outR45_2000;


extern const t_pid_gain sp_gain_turnIn135_2000;;
extern const t_pid_gain om_gain_turnIn135_2000;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_2000_table;
extern const t_turn_param_table slalom_inR135_2000_table;
extern const t_param param_inL135_2000;
extern const t_param param_inR135_2000;

//
extern const t_pid_gain sp_gain_turnOut135_2000;;
extern const t_pid_gain om_gain_turnOut135_2000;
extern const t_turn_param_table slalom_outL135_2000_table;
extern const t_turn_param_table slalom_outR135_2000_table;
extern const t_param param_outL135_2000;
extern const t_param param_outR135_2000;

extern const t_param *const mode_2000[17];

extern const t_param *const mode_2000_acc[19];
extern const t_param *const mode_2000_acc_v2[17];




#endif /* CPP_PARAMS_TURN_2000_H_ */
