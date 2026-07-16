/*
 * turn_700.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_700_H_
#define CPP_PARAMS_TURN_700_H_

#include "typedef_run_param.h"

// Identified from the 700 mm/s turn logs captured on 2026-07-15.
// Translational gains are shared; angular gains are fitted per turn direction.
const static t_ff_gain ff_gain_turn_R_700 = {0.8402035f, 0.1023965f, 0.05932111f, 0.006272998f, 0.001307563f, 0.001307563f, 0.0f};
const static t_ff_gain ff_gain_turn_L_700 = {0.8402035f, 0.1023965f, 0.05932111f, 0.008161633f, 0.001264778f, 0.001264778f, 0.0f};

// Identified jointly from the 500 and 700 mm/s turn logs. Velocity and
// acceleration coefficients use both speed conditions, while the fitted bias
// is separated by speed; only the 700 mm/s bias is stored here.
const static t_ff_gain ff_gain_long_turn_R_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.0006231482f, 0.001185694f, 0.001185694f, 0.06181237f
};
const static t_ff_gain ff_gain_long_turn_L_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.004464340f, 0.001461646f, 0.001461646f, 0.02024842f
};
const static t_ff_gain ff_gain_long_turn_180_R_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.005721272f, 0.001043587f, 0.001043587f, 0.01295160f
};
const static t_ff_gain ff_gain_long_turn_180_L_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.006079761f, 0.0009266070f, 0.0009266070f, 0.06087458f
};
// The joint 500/700 fits for 45-degree and V90 turns produce negative
// angular-velocity gains. Constrain those terms to zero and refit angular
// acceleration and the speed-specific signed biases.
const static t_ff_gain ff_gain_in_out_45_R_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.0f, 0.001417219f, 0.001417219f, 0.04041701f
};
const static t_ff_gain ff_gain_in_out_45_L_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.0f, 0.001390124f, 0.001390124f, 0.07084210f
};
const static t_ff_gain ff_gain_in_out_135_R_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.001214761f, 0.001018063f, 0.001018063f, 0.08342324f
};
const static t_ff_gain ff_gain_in_out_135_L_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.002087457f, 0.001058758f, 0.001058758f, 0.1018680f
};
const static t_ff_gain ff_gain_v90_R_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.0f, 0.001258248f, 0.001258248f, 0.07607347f
};
const static t_ff_gain ff_gain_v90_L_700 = {
	0.8700466f, 0.1147426f, 0.07730041f, 0.0f, 0.001265949f, 0.001265949f, 0.1155889f
};


//-----------velo = 700 mm/s parameters
const static t_pid_gain sp_gain_turn90_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_L90_700_table = {0.70f, 42.5f,29.97,38.86, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_700_table = {0.70f,-42.5f,29.97,38.86,-90.0f,Turn_R};// k= 100
const static t_param param_L90_700 = {&slalom_L90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700,&ff_gain_long_turn_L_700};
const static t_param param_R90_700 = {&slalom_R90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700,&ff_gain_long_turn_R_700};

const static t_pid_gain sp_gain_turn180_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_L180_700_table = {0.70f, 43.5f,19.28,30.11, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_700_table = {0.70f,-43.5f,19.28,30.11,-180.0f,Turn_R};
const static t_param param_L180_700 = {&slalom_L180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700,&ff_gain_long_turn_180_L_700};
const static t_param param_R180_700 = {&slalom_R180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700,&ff_gain_long_turn_180_R_700};

const static t_pid_gain sp_gain_turnV90_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_LV90_700_table = {0.70f, 38.50f,9.18,18.18, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_700_table = {0.70f,-38.50f,9.18,18.18,-90.0f,Turn_R};
const static t_param param_LV90_700 = {&slalom_LV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700,&ff_gain_v90_L_700};
const static t_param param_RV90_700 = {&slalom_RV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700,&ff_gain_v90_R_700};

const static t_pid_gain sp_gain_turnIn45_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_inL45_700_table = {0.70f, 48.5f,13.12,40.31, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_700_table = {0.70f,-48.5f,13.12,40.31,-45.0f,Turn_R};
const static t_param param_inL45_700 = {&slalom_inL45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700,&ff_gain_in_out_45_L_700};
const static t_param param_inR45_700 = {&slalom_inR45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700,&ff_gain_in_out_45_R_700};

const static t_pid_gain sp_gain_turnOut45_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL45_700_table = {0.70f, 48.5f,31.81,24.64, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_700_table = {0.70f,-48.5f,31.81,24.64,-45.0f,Turn_R};
const static t_param param_outL45_700 = {&slalom_outL45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700,&ff_gain_in_out_45_L_700};
const static t_param param_outR45_700 = {&slalom_outR45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700,&ff_gain_in_out_45_R_700};

const static t_pid_gain sp_gain_turnIn135_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_700 = {0.10f,0.01f,0.0f};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_700_table = {0.70f, 40.5f,13.29,14.42, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_700_table = {0.70f,-40.5f,13.29,14.42,-135.0f,Turn_R};
const static t_param param_inL135_700 = {&slalom_inL135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700,&ff_gain_in_out_135_L_700};
const static t_param param_inR135_700 = {&slalom_inR135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700,&ff_gain_in_out_135_R_700};

const static t_pid_gain sp_gain_turnOut135_700 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_700 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL135_700_table = {0.70f, 40.5f,5.62,22.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_700_table = {0.70f,-40.5f,5.62,22.17,-135.0f,Turn_R};
const static t_param param_outL135_700 = {&slalom_outL135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700,&ff_gain_in_out_135_L_700};
const static t_param param_outR135_700 = {&slalom_outR135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700,&ff_gain_in_out_135_R_700};

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
