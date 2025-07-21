/*
 * turn_700.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_700_H_
#define CPP_PARAMS_TURN_700_H_

#include "typedef_run_param.h"

//-----------velo = 500 mm/s parameters
const static t_pid_gain sp_gain_turn90_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_700 = {0.8, 0.002, 0.0};
const static t_turn_param_table slalom_L90_700_table = {0.70f, 42.5f,29.97,38.86, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_700_table = {0.70f,-42.5f,29.97,38.86,-90.0f,Turn_R};// k= 100
const static t_param param_L90_700 = {&slalom_L90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};
const static t_param param_R90_700 = {&slalom_R90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};

const static t_pid_gain sp_gain_turn180_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_700 = {0.8, 0.02, 0.0};
const static t_turn_param_table slalom_L180_700_table = {0.70f, 43.5f,19.28,30.11, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_700_table = {0.70f,-43.5f,19.28,30.11,-180.0f,Turn_R};
const static t_param param_L180_700 = {&slalom_L180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};
const static t_param param_R180_700 = {&slalom_R180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};

const static t_pid_gain sp_gain_turnV90_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_700 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_LV90_700_table = {0.70f, 38.50f,9.18,18.18, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_700_table = {0.70f,-38.50f,9.18,18.18,-90.0f,Turn_R};
const static t_param param_LV90_700 = {&slalom_LV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};
const static t_param param_RV90_700 = {&slalom_RV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};

const static t_pid_gain sp_gain_turnIn45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_700 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_inL45_700_table = {0.70f, 48.5f,13.12,40.31, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_700_table = {0.70f,-48.5f,13.12,40.31,-45.0f,Turn_R};
const static t_param param_inL45_700 = {&slalom_inL45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};
const static t_param param_inR45_700 = {&slalom_inR45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};

const static t_pid_gain sp_gain_turnOut45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_700 = {0.4, 0.02, 0.0};;
const static t_turn_param_table slalom_outL45_700_table = {0.70f, 48.5f,31.81,24.64, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_700_table = {0.70f,-48.5f,31.81,24.64,-45.0f,Turn_R};
const static t_param param_outL45_700 = {&slalom_outL45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};
const static t_param param_outR45_700 = {&slalom_outR45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};

const static t_pid_gain sp_gain_turnIn135_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_700 = {0.8, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_700_table = {0.70f, 40.5f,13.29,14.42, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_700_table = {0.70f,-40.5f,13.29,14.42,-135.0f,Turn_R};
const static t_param param_inL135_700 = {&slalom_inL135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};
const static t_param param_inR135_700 = {&slalom_inR135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};

const static t_pid_gain sp_gain_turnOut135_700 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_700 = {0.8, 0.02, 0.0};
const static t_turn_param_table slalom_outL135_700_table = {0.70f, 40.5f,5.62,22.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_700_table = {0.70f,-40.5f,5.62,22.17,-135.0f,Turn_R};
const static t_param param_outL135_700 = {&slalom_outL135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};
const static t_param param_outR135_700 = {&slalom_outR135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};

const static t_param *const mode_700[] = 	{	NULL,					NULL,			NULL,
												&param_R90_700,		&param_L90_700,
												&param_R180_700,	&param_L180_700,
												&param_inR45_700,	&param_inL45_700,
												&param_outR45_700,	&param_outL45_700,
												&param_inR135_700,	&param_inL135_700,
												&param_outR135_700,	&param_outL135_700,
												&param_RV90_700,	&param_LV90_700
											};


#endif /* CPP_PARAMS_TURN_700_H_ */
