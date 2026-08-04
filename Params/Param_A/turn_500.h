/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "Params/typedef_run_param.h"

// 500 mm/s turn-specific feedforward gains. Angular jerk gains are constrained
// to non-negative values and are fitted independently for each turn shape.
const static t_ff_gain ff_gain_long_turn_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002077371f, 0.000836273f, 0.000930115f, 0.08559123f, 0.000001569006f
};
const static t_ff_gain ff_gain_long_turn_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003051475f, 0.000741268f, 0.000965438f, 0.08559123f, 0.000001971694f
};
const static t_ff_gain ff_gain_long_turn_180_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002126270f, 0.000531106f, 0.001443002f, 0.08559123f, 0.0f
};
const static t_ff_gain ff_gain_long_turn_180_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002432468f, 0.000538646f, 0.001405904f, 0.08559123f, 0.0f
};
const static t_ff_gain ff_gain_in_out_45_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.000849904f, 0.000794332f, 0.000776772f, 0.08559123f, 0.000003679442f
};
const static t_ff_gain ff_gain_in_out_45_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.001129579f, 0.000934826f, 0.000928198f, 0.08559123f, 0.000003962296f
};
const static t_ff_gain ff_gain_in_out_135_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003190175f, 0.001071977f, 0.000987594f, 0.08559123f, 0.000000389595f
};
const static t_ff_gain ff_gain_in_out_135_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003156581f, 0.001066273f, 0.000982570f, 0.08559123f, 0.000000178623f
};
const static t_ff_gain ff_gain_v90_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.001678876f, 0.000675940f, 0.001023082f, 0.08559123f, 0.000000758492f
};
const static t_ff_gain ff_gain_v90_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002037948f, 0.000650298f, 0.001040909f, 0.08559123f, 0.000001038299f
};

const static t_pid_gain sp_gain_turn90_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500,&ff_gain_long_turn_L_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500,&ff_gain_long_turn_R_500};

const static t_pid_gain sp_gain_turn180_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500,&ff_gain_long_turn_180_L_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500,&ff_gain_long_turn_180_R_500};

const static t_pid_gain sp_gain_turnV90_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_L_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_R_500};

const static t_pid_gain sp_gain_turnIn45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_L_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnOut45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_L_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnIn135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_L_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_R_500};

const static t_pid_gain sp_gain_turnOut135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500,&ff_gain_in_out_135_L_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500,&ff_gain_in_out_135_R_500};

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
