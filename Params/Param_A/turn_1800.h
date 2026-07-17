/*
 * turn_1800.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1800_H_
#define CPP_PARAMS_TURN_1800_H_

#include "typedef_run_param.h"


const static t_ff_gain ff_gain_long_turn_90_R_1800 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.002f, 0.00096f, 0.00098f, 0.2972200f
};
const static t_ff_gain ff_gain_long_turn_90_L_1800 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.002f, 0.00085f, 0.00106f, 0.43f
};
const static t_ff_gain ff_gain_long_turn_180_R_1800 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.007355744f, 0.0009893826f, 0.0009893826f, 0.1964514f
};
const static t_ff_gain ff_gain_long_turn_180_L_1800 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.008186232f, 0.0009810843f, 0.0009810843f, 0.1963813f
};
const static t_ff_gain ff_gain_in45_R_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.000825f, 0.000625f, 0.0f
};
const static t_ff_gain ff_gain_in45_L_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.00092f, 0.00070f, 0.04f
};
const static t_ff_gain ff_gain_out45_R_1800 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0f, 0.000825f, 0.000625f, 0.0f
};
const static t_ff_gain ff_gain_out45_L_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.00092f, 0.00070f, 0.04f
};
const static t_ff_gain ff_gain_in135_R_1800 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.004f, 0.001069892f, 0.001069892f, 0.3761218f
};
const static t_ff_gain ff_gain_in135_L_1800 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.004f, 0.00086f, 0.00113f, 0.485f
};
const static t_ff_gain ff_gain_out135_R_1800 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.004f, 0.001069892f, 0.001069892f, 0.3761218f
};
const static t_ff_gain ff_gain_out135_L_1800 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.004f, 0.00086f, 0.00113f, 0.485f
};
const static t_ff_gain ff_gain_v90_R_1800 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0f, 0.00085f, 0.00065f, 0.250f
};
const static t_ff_gain ff_gain_v90_L_1800 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0f, 0.00091f, 0.00102f, 0.40f
};


//k = 250
const static t_pid_gain sp_gain_turn90_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turn90_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_L90_1800_table = {1.80f, 52.0f,13.71,38.14, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1800_table = {1.80f,-52.0f,13.71,38.14,-90.0f,Turn_R};
const static t_param param_L90_1800 = {&slalom_L90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800,&ff_gain_long_turn_90_L_1800};
const static t_param param_R90_1800 = {&slalom_R90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800,&ff_gain_long_turn_90_R_1800};
//k = 250
const static t_pid_gain sp_gain_turn180_1800 = {2.0,0.2,0.0};
const static t_pid_gain om_gain_turn180_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,9.90,39.15, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,9.90,39.15,-180.0f,Turn_R};
const static t_param param_L180_1800 = {&slalom_L180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800,&ff_gain_long_turn_180_L_1800};
const static t_param param_R180_1800 = {&slalom_R180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800,&ff_gain_long_turn_180_R_1800};
//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnV90_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_LV90_1800_table = {1.80f, 40.0f,3.10,27.38+00.0, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1800_table = {1.80f,-40.0f,3.10,27.38+00.0,-90.0f,Turn_R};
const static t_param param_LV90_1800 = {&slalom_LV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800,&ff_gain_v90_L_1800};
const static t_param param_RV90_1800 = {&slalom_RV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800,&ff_gain_v90_R_1800};
//k = 250
const static t_pid_gain sp_gain_turnIn45_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnIn45_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_inL45_1800_table = {1.80f, 53.0f,9.27,41.09, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1800_table = {1.80f,-53.0f,9.27,41.09,-45.0f,Turn_R};
const static t_param param_inL45_1800 = {&slalom_inL45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800,&ff_gain_in45_L_1800};
const static t_param param_inR45_1800 = {&slalom_inR45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800,&ff_gain_in45_R_1800};

//k = 250
const static t_pid_gain sp_gain_turnOut45_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnOut45_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.81,22.01, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,23.81,22.01,-45.0f,Turn_R};
const static t_param param_outL45_1800 = {&slalom_outL45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800,&ff_gain_out45_L_1800};
const static t_param param_outR45_1800 = {&slalom_outR45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800,&ff_gain_out45_R_1800};

//k = 250
const static t_pid_gain sp_gain_turnIn135_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnIn135_1800 = {0.1,0.002,0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1800_table = {1.80f, 43.0f,10.67,31.00, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1800_table = {1.80f,-43.0f,10.67,31.00,-135.0f,Turn_R};
const static t_param param_inL135_1800 = {&slalom_inL135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800,&ff_gain_in135_L_1800};
const static t_param param_inR135_1800 = {&slalom_inR135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800,&ff_gain_in135_R_1800};

//k = 250
const static t_pid_gain sp_gain_turnOut135_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_turnOut135_1800 = {0.1,0.002,0.0};
const static t_turn_param_table slalom_outL135_1800_table = {1.80f, 41.0f,9.30,44.12, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1800_table = {1.80f,-41.0f,9.30,44.12,-135.0f,Turn_R};
const static t_param param_outL135_1800 = {&slalom_outL135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800,&ff_gain_out135_L_1800};
const static t_param param_outR135_1800 = {&slalom_outR135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800,&ff_gain_out135_R_1800};

const static t_param *const mode_1800[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,	&param_L180_1800,
												&param_inR45_1800,	&param_inL45_1800,
												&param_outR45_1800,	&param_outL45_1800,
												&param_inR135_1800,	&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												&param_RV90_1800,	&param_LV90_1800
											};

const static t_param *const mode_1800_acc_v1[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,		&param_L180_1800,
												NULL,					NULL	,
												NULL,					NULL	,
												&param_inR135_1800,		&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												NULL,					NULL	,
												NULL,					NULL
											};

const static t_param *const mode_1800_acc_v2[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,		&param_L180_1800,
												&param_inR45_1800,		&param_inL45_1800,
												&param_outR45_1800,		&param_outL45_1800,
												&param_inR135_1800,		&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												NULL,					NULL	,
												NULL,					NULL
											};



#endif /* CPP_PARAMS_TURN_1800_H_ */
