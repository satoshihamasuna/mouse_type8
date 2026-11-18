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

//k = 250
extern const t_pid_gain sp_gain_turn90_1400;
extern const t_pid_gain om_gain_turn90_1400;
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,22.01,38.31, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,22.01,38.31,-90.0f,Turn_R};
extern const t_turn_param_table slalom_L90_1400_table;
extern const t_turn_param_table slalom_R90_1400_table;
extern const t_param param_L90_1400;
extern const t_param param_R90_1400;

//k = 250
extern const t_pid_gain sp_gain_turn180_1400;
extern const t_pid_gain om_gain_turn180_1400;
extern const t_turn_param_table slalom_L180_1400_table;
extern const t_turn_param_table slalom_R180_1400_table;
extern const t_param param_L180_1400;
extern const t_param param_R180_1400;

//not adjust
//k = 250
extern const t_pid_gain sp_gain_turnV90_1400;
extern const t_pid_gain om_gain_turnV90_1400;
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,11.55-2.0,26.99, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,11.55-2.0,26.99,-90.0f,Turn_R};
extern const t_turn_param_table slalom_LV90_1400_table;
extern const t_turn_param_table slalom_RV90_1400_table;
extern const t_param param_LV90_1400;
extern const t_param param_RV90_1400;

//k = 250
extern const t_pid_gain sp_gain_turnIn45_1400;
extern const t_pid_gain om_gain_turnIn45_1400;
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,8.28,40.77, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.28,40.77,-45.0f,Turn_R};
extern const t_turn_param_table slalom_inL45_1400_table;
extern const t_turn_param_table slalom_inR45_1400_table;
extern const t_param param_inL45_1400;
extern const t_param param_inR45_1400;

//k = 220
//k = 250
extern const t_pid_gain sp_gain_turnOut45_1400;
extern const t_pid_gain om_gain_turnOut45_1400;
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,26.22,22.11, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,26.22,22.11,-45.0f,Turn_R};
extern const t_turn_param_table slalom_outL45_1400_table;
extern const t_turn_param_table slalom_outR45_1400_table;
extern const t_param param_outL45_1400;
extern const t_param param_outR45_1400;

//k= 250
extern const t_pid_gain sp_gain_turnIn135_1400;
extern const t_pid_gain om_gain_turnIn135_1400;//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,21.48,31.96, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,21.48,31.86,-135.0f,Turn_R};
extern const t_turn_param_table slalom_inL135_1400_table;
extern const t_turn_param_table slalom_inR135_1400_table;
extern const t_param param_inL135_1400;
extern const t_param param_inR135_1400;

//220
extern const t_pid_gain sp_gain_turnOut135_1400;
extern const t_pid_gain om_gain_turnOut135_1400;
//const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 42.0f,13.88,39.77, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-42.0f,13.88,39.77,-135.0f,Turn_R};
extern const t_turn_param_table slalom_outL135_1400_table;
extern const t_turn_param_table slalom_outR135_1400_table;
extern const t_param param_outL135_1400;
extern const t_param param_outR135_1400;

extern const t_param *const mode_1400[17];

extern const t_param *const mode_1400_acc[19];
#endif /* CPP_PARAMS_TURN_1400_H_ */
