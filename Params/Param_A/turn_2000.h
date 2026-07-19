/*
 * turn_2000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_2000_H_
#define CPP_PARAMS_TURN_2000_H_

#include "typedef_run_param.h"

// OM feedforward for every turn is seeded from the verified L90-L result.
// Dedicated variables are retained for later per-turn/direction tuning;
// turn-specific SP feedforward remains unchanged.
const static t_ff_gain ff_gain_long_turn_90_R_2000 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_long_turn_90_L_2000 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_long_turn_180_R_2000 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_long_turn_180_L_2000 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_in45_R_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_in45_L_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_out45_R_2000 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_out45_L_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_in135_R_2000 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_in135_L_2000 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_out135_R_2000 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_out135_L_2000 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_v90_R_2000 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};
const static t_ff_gain ff_gain_v90_L_2000 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0188f, 0.00105f, 0.00110f, 0.0175f, 0.0000030f
};


const static t_pid_gain sp_gain_turn90_2000 = {2.0,0.016,0.0};
const static t_pid_gain om_gain_turn90_2000 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
const static t_param param_L90_2000 = {&slalom_L90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000,&ff_gain_long_turn_90_L_2000};
const static t_param param_R90_2000 = {&slalom_R90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000,&ff_gain_long_turn_90_R_2000};

const static t_pid_gain sp_gain_turn180_2000 = {2.0,0.1,0.00};
const static t_pid_gain om_gain_turn180_2000 = {0.10,0.002,0.0};
const static t_turn_param_table slalom_L180_2000_table = {2.00f, 50.0f,5.28,41.08, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_2000_table = {2.00f,-50.0f,5.28,41.08,-180.0f,Turn_R};
const static t_param param_L180_2000 = {&slalom_L180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000,&ff_gain_long_turn_180_L_2000};
const static t_param param_R180_2000 = {&slalom_R180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000,&ff_gain_long_turn_180_R_2000};


//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_2000 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnV90_2000 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_LV90_2000_table = {2.00f, 38.0f,1.8,35, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_2000_table = {2.00f,-38.0f,1.8,35,-90.0f,Turn_R};
const static t_param param_LV90_2000 = {&slalom_LV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000,&ff_gain_v90_L_2000};
const static t_param param_RV90_2000 = {&slalom_RV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000,&ff_gain_v90_R_2000};
//k = 300
const static t_pid_gain sp_gain_turnIn45_2000 = {2.0,0.051,0.0};
const static t_pid_gain om_gain_turnIn45_2000 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_inL45_2000_table = {2.00f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_2000_table = {2.00f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const static t_param param_inL45_2000 = {&slalom_inL45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000,&ff_gain_in45_L_2000};
const static t_param param_inR45_2000 = {&slalom_inR45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000,&ff_gain_in45_R_2000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_2000 = {2.0,0.051,0.0};
const static t_pid_gain om_gain_turnOut45_2000 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_outL45_2000_table = {2.00f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_2000_table = {2.00f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const static t_param param_outL45_2000 = {&slalom_outL45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000,&ff_gain_out45_L_2000};
const static t_param param_outR45_2000 = {&slalom_outR45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000,&ff_gain_out45_R_2000};


const static t_pid_gain sp_gain_turnIn135_2000 = {2.0,0.051,0.0};
const static t_pid_gain om_gain_turnIn135_2000 = {0.1,0.002,0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_2000_table = {2.00f, 42.0f,8.0,30.0, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_2000_table = {2.00f,-42.0f,8.0,30.0,-135.0f,Turn_R};
const static t_param param_inL135_2000 = {&slalom_inL135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000,&ff_gain_in135_L_2000};
const static t_param param_inR135_2000 = {&slalom_inR135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000,&ff_gain_in135_R_2000};

//
const static t_pid_gain sp_gain_turnOut135_2000 = {2.0,0.051,0.0};
const static t_pid_gain om_gain_turnOut135_2000 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_outL135_2000_table = {2.00f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_2000_table = {2.00f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const static t_param param_outL135_2000 = {&slalom_outL135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000,&ff_gain_out135_L_2000};
const static t_param param_outR135_2000 = {&slalom_outR135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000,&ff_gain_out135_R_2000};

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
const static t_param *const mode_2000_acc_v2[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,	&param_L180_2000,
												&param_inR45_2000,	&param_inL45_2000,
												&param_outR45_2000,	&param_outL45_2000,
												&param_inR135_2000,	&param_inL135_2000,
												&param_outR135_2000,	&param_outL135_2000,
												NULL,					NULL
											};




#endif /* CPP_PARAMS_TURN_2000_H_ */
