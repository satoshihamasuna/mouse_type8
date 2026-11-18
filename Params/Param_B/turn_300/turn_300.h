/*
 * turn_300.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_300_H_
#define CPP_PARAMS_TURN_300_H_

// 値の定義は同じディレクトリのturn_300.cppにあります。

#include "Params/typedef_run_param.h"


extern const t_pid_gain sp_gain_turn90_300;
extern const t_pid_gain om_gain_turn90_300;
extern const t_turn_param_table slalom_L90_300_table;
extern const t_turn_param_table slalom_R90_300_table;
extern const t_param param_L90_300;
extern const t_param param_R90_300;

extern const t_pid_gain sp_gain_turn180_300;
extern const t_pid_gain om_gain_turn180_300;
extern const t_turn_param_table slalom_L180_300_table;
extern const t_turn_param_table slalom_R180_300_table;
extern const t_param param_L180_300;
extern const t_param param_R180_300;

extern const t_pid_gain sp_gain_turnV90_300;
extern const t_pid_gain om_gain_turnV90_300;
extern const t_turn_param_table slalom_LV90_300_table;
extern const t_turn_param_table slalom_RV90_300_table;
extern const t_param param_LV90_300;
extern const t_param param_RV90_300;

extern const t_pid_gain sp_gain_turnIn45_300;
extern const t_pid_gain om_gain_turnIn45_300;
extern const t_turn_param_table slalom_inL45_300_table;
extern const t_turn_param_table slalom_inR45_300_table;
extern const t_param param_inL45_300;
extern const t_param param_inR45_300;

extern const t_pid_gain sp_gain_turnOut45_300;
extern const t_pid_gain om_gain_turnOut45_300;
extern const t_turn_param_table slalom_outL45_300_table;
extern const t_turn_param_table slalom_outR45_300_table;
extern const t_param param_outL45_300;
extern const t_param param_outR45_300;

extern const t_pid_gain sp_gain_turnIn135_300;
extern const t_pid_gain om_gain_turnIn135_300;
extern const t_turn_param_table slalom_inL135_300_table;
extern const t_turn_param_table slalom_inR135_300_table;
extern const t_param param_inL135_300;
extern const t_param param_inR135_300;

extern const t_pid_gain sp_gain_turnOut135_300;
extern const t_pid_gain om_gain_turnOut135_300;
extern const t_turn_param_table slalom_outL135_300_table;
extern const t_turn_param_table slalom_outR135_300_table;
extern const t_param param_outL135_300;
extern const t_param param_outR135_300;

extern const t_param *const mode_300[17];



#endif /* CPP_PARAMS_TURN_300_H_ */
