/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "Params/typedef_run_param.h"

// 2026-11-14 Mouse B: UNTESTED geometry seeds from Mouse A turn_500.
// Candidate only: B PID and existing FF fallback are retained.
// Do not treat this file as a measured/adopted Mouse B table.
// See candidates.json and README.md in this folder.

const static t_pid_gain sp_gain_turn90_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.10f,0.002f,0.0f};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,42.0f,33.7f, 90.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,44.0f,35.0f,-90.0f,Turn_R,0.0f};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.10f,0.002f,0.0f};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 44.7f,23.63f,16.3f, 180.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-45.0f,23.63f,16.3f,-180.0f,Turn_R,0.0f};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f,41.0f,9.3f,5.1f,90.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,10.6f,3.6f,-90.0f,Turn_R,0.5f};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f,50.0f,20.0f,25.0f,45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,15.3f,27.9f,-45.0f,Turn_R,0.5f};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f,50.0f,33.9f,11.8f,45.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,37.2f,9.2f,-45.0f,Turn_R,0.5f};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f,30.0f,48.2f,36.5f,135.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,47.8f,38.35f,-135.0f,Turn_R,0.5f};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f,30.0f,42.9f,46.07f,135.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,40.7f,46.07f,-135.0f,Turn_R,0.5f};
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
