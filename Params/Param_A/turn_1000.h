/*
 * turn_1000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1000_H_
#define CPP_PARAMS_TURN_1000_H_

#include "typedef_run_param.h"

//k = 200

const static t_pid_gain sp_gain_turn90_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1000_table = {1.00f, 42.5f,29.48,39.30, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1000_table = {1.00f,-42.5f,29.48,39.30,-90.0f,Turn_R};
const static t_param param_L90_1000 = {&slalom_L90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};
const static t_param param_R90_1000 = {&slalom_R90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};

const static t_pid_gain sp_gain_turn180_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1000_table = {1.00f, 45.0f,18.54,28.49, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1000_table = {1.00f,-45.0f,18.54,28.49,-180.0f,Turn_R};
const static t_param param_L180_1000 = {&slalom_L180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};
const static t_param param_R180_1000 = {&slalom_R180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};

//not adjust
const static t_pid_gain sp_gain_turnV90_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1000_table = {1.00f, 37.5f,10.20,19.62, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1000_table = {1.00f,-37.5f,10.20,19.62,-90.0f,Turn_R};
const static t_param param_LV90_1000 = {&slalom_LV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};
const static t_param param_RV90_1000 = {&slalom_RV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};

const static t_pid_gain sp_gain_turnIn45_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_1000_table = {1.00f, 45.0f,15.52,42.41, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1000_table = {1.00f,-45.0f,15.52,42.41,-45.0f,Turn_R};
const static t_param param_inL45_1000 = {&slalom_inL45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};
const static t_param param_inR45_1000 = {&slalom_inR45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_1000_table = {1.00f, 50.0f,31.04,21.30, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1000_table = {1.00f,-50.0f,31.04,21.30,-45.0f,Turn_R};
const static t_param param_outL45_1000 = {&slalom_outL45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};
const static t_param param_outR45_1000 = {&slalom_outR45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};


const static t_pid_gain sp_gain_turnIn135_1000 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1000 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1000_table = {1.00f, 37.5f,27.18,29.87, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1000_table = {1.00f,-37.5f,27.18,29.87,-135.0f,Turn_R};
const static t_param param_inL135_1000 = {&slalom_inL135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};
const static t_param param_inR135_1000 = {&slalom_inR135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};

//
const static t_pid_gain sp_gain_turnOut135_1000 = {10.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1000_table = {1.00f, 38.0f,18.00,35.81, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1000_table = {1.00f,-38.0f,18.00,35.81,-135.0f,Turn_R};
const static t_param param_outL135_1000 = {&slalom_outL135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};
const static t_param param_outR135_1000 = {&slalom_outR135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};

const static t_param *const mode_1000[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1000,		&param_L90_1000,
												&param_R180_1000,	&param_L180_1000,
												&param_inR45_1000,	&param_inL45_1000,
												&param_outR45_1000,	&param_outL45_1000,
												&param_inR135_1000,	&param_inL135_1000,
												&param_outR135_1000,	&param_outL135_1000,
												&param_RV90_1000,	&param_LV90_1000
											};

#endif /* CPP_PARAMS_TURN_1000_H_ */
