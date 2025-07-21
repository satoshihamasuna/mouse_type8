/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.6, 0.005, 0.0};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

const static t_param *const mode_500[] = 	{	NULL,					NULL,			NULL,
												&param_R90_500,		&param_L90_500,
												&param_R180_500,	&param_L180_500,
												&param_inR45_500,	&param_inL45_500,
												&param_outR45_500,	&param_outL45_500,
												&param_inR135_500,	&param_inL135_500,
												&param_outR135_500,	&param_outL135_500,
												&param_RV90_500,	&param_LV90_500
											};




#endif /* CPP_PARAMS_TURN_500_H_ */
