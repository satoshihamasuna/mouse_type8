/*
 * turn_300.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_300_H_
#define CPP_PARAMS_TURN_300_H_

#include "typedef_run_param.h"


const static t_pid_gain sp_gain_turn90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_300_table = {0.30f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_300_table = {0.30f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_300 = {&slalom_L90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};
const static t_param param_R90_300 = {&slalom_R90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};

const static t_pid_gain sp_gain_turn180_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_L180_300_table = {0.30f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_300_table = {0.30f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_300 = {&slalom_L180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};
const static t_param param_R180_300 = {&slalom_R180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};

const static t_pid_gain sp_gain_turnV90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_300_table = {0.30f, 30.0f,22.78,23.50, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_300_table = {0.30f,-30.0f,22.78,23.50,-90.0f,Turn_R};
const static t_param param_LV90_300 = {&slalom_LV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};
const static t_param param_RV90_300 = {&slalom_RV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};

const static t_pid_gain sp_gain_turnIn45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_300_table = {0.30f, 30.0f,27.04,46.34, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_300_table = {0.30f,-30.0f,27.04,46.34,-45.0f,Turn_R};
const static t_param param_inL45_300 = {&slalom_inL45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};
const static t_param param_inR45_300 = {&slalom_inR45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};

const static t_pid_gain sp_gain_turnOut45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_300_table = {0.30f, 30.0f,45.68,27.70, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_300_table = {0.30f,-30.0f,45.68,27.70,-45.0f,Turn_R};
const static t_param param_outL45_300 = {&slalom_outL45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};
const static t_param param_outR45_300 = {&slalom_outR45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};

const static t_pid_gain sp_gain_turnIn135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL135_300_table = {0.30f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_300_table = {0.30f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_300 = {&slalom_inL135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};
const static t_param param_inR135_300 = {&slalom_inR135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};

const static t_pid_gain sp_gain_turnOut135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_300_table = {0.30f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_300_table = {0.30f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_300 = {&slalom_outL135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};
const static t_param param_outR135_300 = {&slalom_outR135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};

const static t_param *const mode_300[] = 	{	NULL,					NULL,			NULL,
												&param_R90_300,		&param_L90_300,
												&param_R180_300,	&param_L180_300,
												&param_inR45_300,	&param_inL45_300,
												&param_outR45_300,	&param_outL45_300,
												&param_inR135_300,	&param_inL135_300,
												&param_outR135_300,	&param_outL135_300,
												&param_RV90_300,	&param_LV90_300
											};



#endif /* CPP_PARAMS_TURN_300_H_ */
