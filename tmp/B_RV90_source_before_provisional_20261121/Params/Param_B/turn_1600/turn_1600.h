/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

// 値の定義は同じディレクトリのturn_1600.cppにあります。

#include "Params/typedef_run_param.h"
//k = 250
extern const t_pid_gain sp_gain_turn90_1600;
extern const t_pid_gain om_gain_turn90_1600;
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.50f,19.56,38.43, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.50f,19.56,38.43,-90.0f,Turn_R};
extern const t_turn_param_table slalom_L90_1600_table;
extern const t_turn_param_table slalom_R90_1600_table;
extern const t_param param_L90_1600;
extern const t_param param_R90_1600;

//k = 250
extern const t_pid_gain sp_gain_turn180_1600;
extern const t_pid_gain om_gain_turn180_1600;
extern const t_turn_param_table slalom_L180_1600_table;
extern const t_turn_param_table slalom_R180_1600_table;
//const static t_turn_param_table slalom_L180_1600_table = {1.60f,  45.0f,(15.10),(34.72), 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f, -45.0f,(15.10),(34.72),-180.0f,Turn_R};
extern const t_param param_L180_1600;
extern const t_param param_R180_1600;

//k = 250
extern const t_pid_gain sp_gain_turnV90_1600;
extern const t_pid_gain om_gain_turnV90_1600;
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,9.50,29.66, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,9.50,29.66,-90.0f,Turn_R};
extern const t_turn_param_table slalom_LV90_1600_table;
extern const t_turn_param_table slalom_RV90_1600_table;
extern const t_param param_LV90_1600;
extern const t_param param_RV90_1600;

//k = 250
extern const t_pid_gain sp_gain_turnIn45_1600;
extern const t_pid_gain om_gain_turnIn45_1600;
extern const t_ff_gain ff_gain_turnIn45_1600;
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 53.0f,9.84,41.94, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-53.0f,9.84,41.94,-45.0f,Turn_R};
extern const t_turn_param_table slalom_inL45_1600_table;
extern const t_turn_param_table slalom_inR45_1600_table;
extern const t_param param_inL45_1600;
extern const t_param param_inR45_1600;

//k = 250
extern const t_pid_gain sp_gain_turnOut45_1600;
extern const t_pid_gain om_gain_turnOut45_1600;
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 55.0f,18.235,20.24, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-55.0f,18.235,20.24,-45.0f,Turn_R};
extern const t_turn_param_table slalom_outL45_1600_table;
extern const t_turn_param_table slalom_outR45_1600_table;
extern const t_param param_outL45_1600;
extern const t_param param_outR45_1600;

//k = 250,alpha = 1.0
extern const t_pid_gain sp_gain_turnIn135_1600;
extern const t_pid_gain om_gain_turnIn135_1600;//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,13.17,26.34, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,13.17,26.34,-135.0f,Turn_R};
extern const t_turn_param_table slalom_inL135_1600_table;
extern const t_turn_param_table slalom_inR135_1600_table;
extern const t_param param_inL135_1600;
extern const t_param param_inR135_1600;

//k = 250
extern const t_pid_gain sp_gain_turnOut135_1600;
extern const t_pid_gain om_gain_turnOut135_1600;
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 42.0f,11.85,40.11, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-42.0f,11.85,40.1,-135.0f,Turn_R};
extern const t_turn_param_table slalom_outL135_1600_table;
extern const t_turn_param_table slalom_outR135_1600_table;
extern const t_param param_outL135_1600;
extern const t_param param_outR135_1600;

extern const t_param *const mode_1600[17];



extern const t_param *const mode_1600_acc_v1[19];

extern const t_param *const mode_1600_acc_v2[19];



#endif /* CPP_PARAMS_TURN_1600_H_ */
