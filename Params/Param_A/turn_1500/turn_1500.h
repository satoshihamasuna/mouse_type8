/*
 * turn_1500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1500_H_
#define CPP_PARAMS_TURN_1500_H_

// 値の定義は同じディレクトリのturn_1500.cppにあります。

#include "Params/typedef_run_param.h"

//k = 250
extern const t_pid_gain sp_gain_turn90_1500;
extern const t_pid_gain om_gain_turn90_1500;
extern const t_turn_param_table slalom_L90_1500_table;
extern const t_turn_param_table slalom_R90_1500_table;
extern const t_param param_L90_1500;
extern const t_param param_R90_1500;

extern const t_pid_gain sp_gain_turn180_1500;
extern const t_pid_gain om_gain_turn180_1500;
extern const t_turn_param_table slalom_L180_1500_table;
extern const t_turn_param_table slalom_R180_1500_table;
//const static t_turn_param_table slalom_L180_1500_table = {1.50f, 46.0f,13.69,33.69, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1500_table = {1.50f,-46.0f,13.69,33.69,-180.0f,Turn_R};
extern const t_param param_L180_1500;
extern const t_param param_R180_1500;
//k = 400
//not adjust
extern const t_pid_gain sp_gain_turnV90_1500;
extern const t_pid_gain om_gain_turnV90_1500;
extern const t_turn_param_table slalom_LV90_1500_table;
extern const t_turn_param_table slalom_RV90_1500_table;
//const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 37.00f,10.85,20.72, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-37.00f,10.85,20.72,-90.0f,Turn_R};
extern const t_param param_LV90_1500;
extern const t_param param_RV90_1500;

//k = 300
extern const t_pid_gain sp_gain_turnIn45_1500;
extern const t_pid_gain om_gain_turnIn45_1500;
//const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 52.50f,9.02,42.23, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-52.50f,9.02,42.23,-45.0f,Turn_R};
extern const t_turn_param_table slalom_inL45_1500_table;
extern const t_turn_param_table slalom_inR45_1500_table;
extern const t_param param_inL45_1500;
extern const t_param param_inR45_1500;

//k = 300
extern const t_pid_gain sp_gain_turnOut45_1500;
extern const t_pid_gain om_gain_turnOut45_1500;
//const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 52.5f,27.75,23.57, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-52.5f,27.75,23.57,-45.0f,Turn_R};
extern const t_turn_param_table slalom_outL45_1500_table;
extern const t_turn_param_table slalom_outR45_1500_table;
extern const t_param param_outL45_1500;
extern const t_param param_outR45_1500;

//k = 250 , Kp = 4.0
extern const t_pid_gain sp_gain_turnIn135_1500;
extern const t_pid_gain om_gain_turnIn135_1500;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_1500_table;
extern const t_turn_param_table slalom_inR135_1500_table;
extern const t_param param_inL135_1500;
extern const t_param param_inR135_1500;

//
extern const t_pid_gain sp_gain_turnOut135_1500;
extern const t_pid_gain om_gain_turnOut135_1500;
extern const t_turn_param_table slalom_outL135_1500_table;
extern const t_turn_param_table slalom_outR135_1500_table;
extern const t_param param_outL135_1500;
extern const t_param param_outR135_1500;

extern const t_param *const mode_1500[17];



#endif /* CPP_PARAMS_TURN_1500_H_ */
