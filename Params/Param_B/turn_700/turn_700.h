/*
 * turn_700.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_700_H_
#define CPP_PARAMS_TURN_700_H_

// 値の定義は同じディレクトリのturn_700.cppにあります。

#include "Params/typedef_run_param.h"

//-----------velo = 500 mm/s parameters
extern const t_pid_gain sp_gain_turn90_700;
extern const t_pid_gain om_gain_turn90_700;
extern const t_turn_param_table slalom_L90_700_table;// k= 100
extern const t_turn_param_table slalom_R90_700_table;// k= 100
extern const t_param param_L90_700;
extern const t_param param_R90_700;

extern const t_pid_gain sp_gain_turn180_700;
extern const t_pid_gain om_gain_turn180_700;
extern const t_turn_param_table slalom_L180_700_table;
extern const t_turn_param_table slalom_R180_700_table;
extern const t_param param_L180_700;
extern const t_param param_R180_700;

extern const t_pid_gain sp_gain_turnV90_700;
extern const t_pid_gain om_gain_turnV90_700;
extern const t_turn_param_table slalom_LV90_700_table;
extern const t_turn_param_table slalom_RV90_700_table;
extern const t_param param_LV90_700;
extern const t_param param_RV90_700;

extern const t_pid_gain sp_gain_turnIn45_700;
extern const t_pid_gain om_gain_turnIn45_700;
extern const t_turn_param_table slalom_inL45_700_table;
extern const t_turn_param_table slalom_inR45_700_table;
extern const t_param param_inL45_700;
extern const t_param param_inR45_700;

extern const t_pid_gain sp_gain_turnOut45_700;
extern const t_pid_gain om_gain_turnOut45_700;
extern const t_turn_param_table slalom_outL45_700_table;
extern const t_turn_param_table slalom_outR45_700_table;
extern const t_param param_outL45_700;
extern const t_param param_outR45_700;

extern const t_pid_gain sp_gain_turnIn135_700;
extern const t_pid_gain om_gain_turnIn135_700;//{0.7f, 0.7f, 0.0f};
extern const t_turn_param_table slalom_inL135_700_table;
extern const t_turn_param_table slalom_inR135_700_table;
extern const t_param param_inL135_700;
extern const t_param param_inR135_700;

extern const t_pid_gain sp_gain_turnOut135_700;
extern const t_pid_gain om_gain_turnOut135_700;
extern const t_turn_param_table slalom_outL135_700_table;
extern const t_turn_param_table slalom_outR135_700_table;
extern const t_param param_outL135_700;
extern const t_param param_outR135_700;

extern const t_param *const mode_700[17];


#endif /* CPP_PARAMS_TURN_700_H_ */
