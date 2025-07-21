/*
 * turn_2000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_2000_H_
#define CPP_PARAMS_TURN_2000_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_2000 = {0.6, 0.06, 0.0};
const static t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
const static t_param param_L90_2000 = {&slalom_L90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};
const static t_param param_R90_2000 = {&slalom_R90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};

const static t_pid_gain sp_gain_turn180_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_2000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_2000_table = {2.00f, 50.0f,5.28,41.08, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_2000_table = {2.00f,-50.0f,5.28,41.08,-180.0f,Turn_R};
const static t_param param_L180_2000 = {&slalom_L180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000};
const static t_param param_R180_2000 = {&slalom_R180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000};



//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_2000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_2000_table = {2.00f, 38.0f,9.50,29.66, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_2000_table = {2.00f,-38.0f,9.50,29.66,-90.0f,Turn_R};
const static t_param param_LV90_2000 = {&slalom_LV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000};
const static t_param param_RV90_2000 = {&slalom_RV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000};
//k = 300
const static t_pid_gain sp_gain_turnIn45_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_2000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_2000_table = {2.00f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_2000_table = {2.00f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const static t_param param_inL45_2000 = {&slalom_inL45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000};
const static t_param param_inR45_2000 = {&slalom_inR45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_2000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_2000_table = {2.00f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_2000_table = {2.00f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const static t_param param_outL45_2000 = {&slalom_outL45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000};
const static t_param param_outR45_2000 = {&slalom_outR45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000};


const static t_pid_gain sp_gain_turnIn135_2000 = {10.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_2000 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_2000_table = {2.00f, 44.0f,13.17,26.34, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_2000_table = {2.00f,-44.0f,13.17,26.34,-135.0f,Turn_R};
const static t_param param_inL135_2000 = {&slalom_inL135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000};
const static t_param param_inR135_2000 = {&slalom_inR135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000};

//
const static t_pid_gain sp_gain_turnOut135_2000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_2000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_2000_table = {2.00f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_2000_table = {2.00f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const static t_param param_outL135_2000 = {&slalom_outL135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000};
const static t_param param_outR135_2000 = {&slalom_outR135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000};

const static t_param *const mode_2000[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,	&param_L180_2000,
												&param_inR45_2000,	&param_inL45_2000,
												&param_outR45_2000,	&param_outL45_2000,
												&param_inR135_2000,	&param_inL135_2000,
												&param_outR135_2000,	&param_outL135_2000,
												&param_RV90_2000,	&param_LV90_2000
											};

const static t_param *const mode_2000_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,		&param_L180_2000,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL
											};




#endif /* CPP_PARAMS_TURN_2000_H_ */
