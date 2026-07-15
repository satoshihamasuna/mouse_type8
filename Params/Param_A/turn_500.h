/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "typedef_run_param.h"

// Identified jointly from the 500 and 700 mm/s turn logs. Velocity and
// acceleration coefficients use both speed conditions, while the fitted bias
// is separated by speed; only the 500 mm/s bias is stored here.
const static t_ff_gain ff_gain_long_turn_R_500 = {
	0.8700466f, 0.1147426f, 0.0006231482f, 0.001185694f, 0.09459957f, 0.08559123f
};
const static t_ff_gain ff_gain_long_turn_L_500 = {
	0.8700466f, 0.1147426f, 0.004464340f, 0.001461646f, 0.09459957f, 0.07266908f
};
const static t_ff_gain ff_gain_long_turn_180_R_500 = {
	0.8700466f, 0.1147426f, 0.005721272f, 0.001043587f, 0.09459957f, 0.02111197f
};
const static t_ff_gain ff_gain_long_turn_180_L_500 = {
	0.8700466f, 0.1147426f, 0.006079761f, 0.0009266070f, 0.09459957f, 0.03580000f
};
// The joint 500/700 fits for 45-degree and V90 turns still produce negative
// angular-velocity gains. Constrain those terms to zero and refit angular
// acceleration and the speed-specific signed biases.
const static t_ff_gain ff_gain_in_out_45_R_500 = {
	0.8700466f, 0.1147426f, 0.0f, 0.001417219f, 0.09459957f, 0.06884993f
};
const static t_ff_gain ff_gain_in_out_45_L_500 = {
	0.8700466f, 0.1147426f, 0.0f, 0.001390124f, 0.09459957f, 0.08803420f
};
const static t_ff_gain ff_gain_in_out_135_R_500 = {
	0.8700466f, 0.1147426f, 0.001214761f, 0.001018063f, 0.09459957f, 0.1123876f
};
const static t_ff_gain ff_gain_in_out_135_L_500 = {
	0.8700466f, 0.1147426f, 0.002087457f, 0.001058758f, 0.09459957f, 0.1230592f
};
const static t_ff_gain ff_gain_v90_R_500 = {
	0.8700466f, 0.1147426f, 0.0f, 0.001258248f, 0.09459957f, 0.06924877f
};
const static t_ff_gain ff_gain_v90_L_500 = {
	0.8700466f, 0.1147426f, 0.0f, 0.001265949f, 0.09459957f, 0.08503230f
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
