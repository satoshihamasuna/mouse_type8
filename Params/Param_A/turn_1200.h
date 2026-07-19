/*
 * turn_1200.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1200_H_
#define CPP_PARAMS_TURN_1200_H_

#include "typedef_run_param.h"

// Long-90 OM feedforward identified from the 20260719 fixed-PID turn-body logs.
const static t_ff_gain ff_gain_long_turn_90_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_long_turn_90_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_long_turn_180_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_long_turn_180_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_v90_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_v90_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_in45_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_in45_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_out45_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_out45_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_in135_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_in135_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_out135_R_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};
const static t_ff_gain ff_gain_out135_L_1200 = {
	0.4239f, 0.09665f, 0.3017f, 0.0170f, 0.00112f, 0.00095f, 0.0175f, 0.0000032f
};

const static t_pid_gain sp_gain_turn90_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_1200 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_L90_1200_table = {1.20f, 46.5f,26.97,38.94, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1200_table = {1.20f,-46.5f,24.56,38.94,-90.0f,Turn_R};
const static t_param param_L90_1200 = {&slalom_L90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200,&ff_gain_long_turn_90_L_1200};
const static t_param param_R90_1200 = {&slalom_R90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200,&ff_gain_long_turn_90_R_1200};

const static t_pid_gain sp_gain_turn180_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_1200 = {0.20,0.01,0.0};

const static t_turn_param_table slalom_L180_1200_table = {1.20f, 49.0f,13.17,27.04, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1200_table = {1.20f,-49.0f,13.17,27.04,-180.0f,Turn_R};

//const static t_turn_param_table slalom_L180_1200_table = {1.20f, 46.0f,18.99,31.27, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1200_table = {1.20f,-46.0f,18.99,31.27,-180.0f,Turn_R};
const static t_param param_L180_1200 = {&slalom_L180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200,&ff_gain_long_turn_180_L_1200};
const static t_param param_R180_1200 = {&slalom_R180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200,&ff_gain_long_turn_180_R_1200};

//not adjust
const static t_pid_gain sp_gain_turnV90_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_1200 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_LV90_1200_table = {1.20f, 41.0f,9.58,21.35, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1200_table = {1.20f,-41.0f,9.58,21.35,-90.0f,Turn_R};
const static t_param param_LV90_1200 = {&slalom_LV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200,&ff_gain_v90_L_1200};
const static t_param param_RV90_1200 = {&slalom_RV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200,&ff_gain_v90_R_1200};

const static t_pid_gain sp_gain_turnIn45_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_1200 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 50.0f,12.52,41.14, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-50.0f,12.52,41.14,-45.0f,Turn_R};
const static t_param param_inL45_1200 = {&slalom_inL45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200,&ff_gain_in45_L_1200};
const static t_param param_inR45_1200 = {&slalom_inR45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200,&ff_gain_in45_R_1200};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_1200 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_outL45_1200_table = {1.20f, 50.0f,31.21,22.48, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1200_table = {1.20f,-50.0f,31.21,22.48,-45.0f,Turn_R};
const static t_param param_outL45_1200 = {&slalom_outL45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200,&ff_gain_out45_L_1200};
const static t_param param_outR45_1200 = {&slalom_outR45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200,&ff_gain_out45_R_1200};


const static t_pid_gain sp_gain_turnIn135_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_1200 = {0.20,0.01,0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1200_table = {1.20f, 40.0f,24.34,30.49, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1200_table = {1.20f,-40.0f,24.34,30.49,-135.0f,Turn_R};
const static t_param param_inL135_1200 = {&slalom_inL135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200,&ff_gain_in135_L_1200};
const static t_param param_inR135_1200 = {&slalom_inR135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200,&ff_gain_in135_R_1200};

//
const static t_pid_gain sp_gain_turnOut135_1200 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_1200 = {0.20,0.01,0.0};
const static t_turn_param_table slalom_outL135_1200_table = {1.20f, 41.0f,16.73,38.29, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1200_table = {1.20f,-41.0f,16.73,38.29,-135.0f,Turn_R};
const static t_param param_outL135_1200 = {&slalom_outL135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200,&ff_gain_out135_L_1200};
const static t_param param_outR135_1200 = {&slalom_outR135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200,&ff_gain_out135_R_1200};

const static t_param *const mode_1200[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1200,		&param_L90_1200,
												&param_R180_1200,	&param_L180_1200,
												&param_inR45_1200,	&param_inL45_1200,
												&param_outR45_1200,	&param_outL45_1200,
												&param_inR135_1200,	&param_inL135_1200,
												&param_outR135_1200,	&param_outL135_1200,
												&param_RV90_1200,	&param_LV90_1200
											};



#endif /* CPP_PARAMS_TURN_1200_H_ */
