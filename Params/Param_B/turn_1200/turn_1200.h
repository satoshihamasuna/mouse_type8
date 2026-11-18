/*
 * turn_1200.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1200_H_
#define CPP_PARAMS_TURN_1200_H_

// 値の定義は同じディレクトリのturn_1200.cppにあります。

#include "Params/typedef_run_param.h"

extern const t_pid_gain sp_gain_turn90_1200;
extern const t_pid_gain om_gain_turn90_1200;
extern const t_turn_param_table slalom_L90_1200_table;
extern const t_turn_param_table slalom_R90_1200_table;
extern const t_param param_L90_1200;
extern const t_param param_R90_1200;

extern const t_pid_gain sp_gain_turn180_1200;
extern const t_pid_gain om_gain_turn180_1200;

extern const t_turn_param_table slalom_L180_1200_table;
extern const t_turn_param_table slalom_R180_1200_table;

//const static t_turn_param_table slalom_L180_1200_table = {1.20f, 46.0f,18.99,31.27, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1200_table = {1.20f,-46.0f,18.99,31.27,-180.0f,Turn_R};
extern const t_param param_L180_1200;
extern const t_param param_R180_1200;

//not adjust
extern const t_pid_gain sp_gain_turnV90_1200;
extern const t_pid_gain om_gain_turnV90_1200;
extern const t_turn_param_table slalom_LV90_1200_table;
extern const t_turn_param_table slalom_RV90_1200_table;
extern const t_param param_LV90_1200;
extern const t_param param_RV90_1200;

extern const t_pid_gain sp_gain_turnIn45_1200;
extern const t_pid_gain om_gain_turnIn45_1200;
extern const t_turn_param_table slalom_inL45_1200_table;
extern const t_turn_param_table slalom_inR45_1200_table;
extern const t_param param_inL45_1200;
extern const t_param param_inR45_1200;

//k = 300
extern const t_pid_gain sp_gain_turnOut45_1200;
extern const t_pid_gain om_gain_turnOut45_1200;
extern const t_turn_param_table slalom_outL45_1200_table;
extern const t_turn_param_table slalom_outR45_1200_table;
extern const t_param param_outL45_1200;
extern const t_param param_outR45_1200;


extern const t_pid_gain sp_gain_turnIn135_1200;
extern const t_pid_gain om_gain_turnIn135_1200;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_1200_table;
extern const t_turn_param_table slalom_inR135_1200_table;
extern const t_param param_inL135_1200;
extern const t_param param_inR135_1200;

//
extern const t_pid_gain sp_gain_turnOut135_1200;
extern const t_pid_gain om_gain_turnOut135_1200;
extern const t_turn_param_table slalom_outL135_1200_table;
extern const t_turn_param_table slalom_outR135_1200_table;
extern const t_param param_outL135_1200;
extern const t_param param_outR135_1200;

extern const t_param *const mode_1200[17];



#endif /* CPP_PARAMS_TURN_1200_H_ */
