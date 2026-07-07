/*
 * turn_1500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1500_H_
#define CPP_PARAMS_TURN_1500_H_

#include "typedef_run_param.h"

//k = 250
const static t_pid_gain sp_gain_turn90_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1500 = {0.3, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1500_table = {1.50f, 55.5f,14.32,32.76, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1500_table = {1.50f,-55.5f,14.32,32.76,-90.0f,Turn_R};
const static t_param param_L90_1500 = {&slalom_L90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};
const static t_param param_R90_1500 = {&slalom_R90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};

const static t_pid_gain sp_gain_turn180_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1500 = {0.3, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1500_table = {1.50f, 52.00f,6.91,28.66, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1500_table = {1.50f,-52.00f,6.91,28.66,-180.0f,Turn_R};
//const static t_turn_param_table slalom_L180_1500_table = {1.50f, 46.0f,13.69,33.69, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1500_table = {1.50f,-46.0f,13.69,33.69,-180.0f,Turn_R};
const static t_param param_L180_1500 = {&slalom_L180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
const static t_param param_R180_1500 = {&slalom_R180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
//k = 400
//not adjust
const static t_pid_gain sp_gain_turnV90_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1500 = {0.3, 0.005, 0.0};
const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 40.0f,6.76,26.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-40.0f,6.76,26.60,-90.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 37.00f,10.85,20.72, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-37.00f,10.85,20.72,-90.0f,Turn_R};
const static t_param param_LV90_1500 = {&slalom_LV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};
const static t_param param_RV90_1500 = {&slalom_RV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};

//k = 300
const static t_pid_gain sp_gain_turnIn45_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1500 = {0.3, 0.005, 0.0};
//const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 52.50f,9.02,42.23, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-52.50f,9.02,42.23,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 55.0f,7.01,37.83, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-55.0f,7.02,37.83,-45.0f,Turn_R};
const static t_param param_inL45_1500 = {&slalom_inL45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};
const static t_param param_inR45_1500 = {&slalom_inR45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1500 = {0.3, 0.005, 0.0};
//const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 52.5f,27.75,23.57, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-52.5f,27.75,23.57,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 55.0f,25.09,17.83, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-55.0f,25.09,17.83,-45.0f,Turn_R};
const static t_param param_outL45_1500 = {&slalom_outL45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};
const static t_param param_outR45_1500 = {&slalom_outR45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};

//k = 250 , Kp = 4.0
const static t_pid_gain sp_gain_turnIn135_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1500 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1500_table = {1.50f, 42.5f,(16.54),26.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1500_table = {1.50f,-42.5f,(16.54),26.17,-135.0f,Turn_R};
const static t_param param_inL135_1500 = {&slalom_inL135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};
const static t_param param_inR135_1500 = {&slalom_inR135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};

//
const static t_pid_gain sp_gain_turnOut135_1500 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1500_table = {1.50f, 42.50f,8.91,34.95, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1500_table = {1.50f,-42.50f,8.91,34.95,-135.0f,Turn_R};
const static t_param param_outL135_1500 = {&slalom_outL135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};
const static t_param param_outR135_1500 = {&slalom_outR135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};

const static t_param *const mode_1500[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1500,		&param_L90_1500,
												&param_R180_1500,	&param_L180_1500,
												&param_inR45_1500,	&param_inL45_1500,
												&param_outR45_1500,	&param_outL45_1500,
												&param_inR135_1500,	&param_inL135_1500,
												&param_outR135_1500,	&param_outL135_1500,
												&param_RV90_1500,	&param_LV90_1500
											};



#endif /* CPP_PARAMS_TURN_1500_H_ */
