/*
 * turn_1400.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1400_H_
#define CPP_PARAMS_TURN_1400_H_

#include "typedef_run_param.h"

// Long-90 OM feedforward identified from the 20260719 fixed-PID turn-body logs.
const static t_ff_gain ff_gain_long_turn_90_R_1400 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00092f, 0.00089f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_long_turn_90_L_1400 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00097f, 0.00088f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_long_turn_180_R_1400 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00088f, 0.00090f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_long_turn_180_L_1400 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00094f, 0.00087f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_in45_R_1400 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00082f, 0.0613f, 0.0000024f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_in45_L_1400 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00097f, 0.00084f, 0.0175f, 0.0000022f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_out45_R_1400 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_out45_L_1400 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_in135_R_1400 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00091f, 0.00090f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_in135_L_1400 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00096f, 0.00088f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_out135_R_1400 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_out135_L_1400 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_v90_R_1400 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00093f, 0.00090f, 0.0613f, 0.0000028f, -0.00010034f, 0.00062859f
};
const static t_ff_gain ff_gain_v90_L_1400 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00099f, 0.00091f, 0.0175f, 0.0000028f, -0.00010034f, 0.00062859f
};

//k = 250
const static t_pid_gain sp_gain_turn90_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,22.01,38.31, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,22.01,38.31,-90.0f,Turn_R};
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,18.88,34.03, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,18.88,34.03,-90.0f,Turn_R};
// Lstart/Lend are the medians of the 20260725 four-times-slow-motion
// Lstart=Lend=0 right-turn marker videos.  Apply the measured right-turn
// values to both directions.
const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,20.52,29.09, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,20.52,29.09,-90.0f,Turn_R};
const static t_param param_L90_1400 = {&slalom_L90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400,&ff_gain_long_turn_90_L_1400};
const static t_param param_R90_1400 = {&slalom_R90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400,&ff_gain_long_turn_90_R_1400};

//k = 250
const static t_pid_gain sp_gain_turn180_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.0f,12.05,28.43, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.0f,12.05,28.43,-180.0f,Turn_R};
const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.0f,24.62,29.51, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.0f,24.62,29.51,-180.0f,Turn_R};
const static t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400,&ff_gain_long_turn_180_L_1400};
const static t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400,&ff_gain_long_turn_180_R_1400};

//not adjust
//k = 250
const static t_pid_gain sp_gain_turnV90_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,11.55-2.0,26.99, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,11.55-2.0,26.99,-90.0f,Turn_R};
const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,3.62,21.38, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,3.62,21.38,-90.0f,Turn_R};
const static t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400,&ff_gain_v90_L_1400};
const static t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400,&ff_gain_v90_R_1400};

//k = 250
const static t_pid_gain sp_gain_turnIn45_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,8.28,40.77, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.28,40.77,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.0f,11.45,42.98, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-50.0f,11.45,42.98,-45.0f,Turn_R};
const static t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400,&ff_gain_in45_L_1400};
const static t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400,&ff_gain_in45_R_1400};

//k = 220
//k = 250
const static t_pid_gain sp_gain_turnOut45_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,26.22,22.11, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,26.22,22.11,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 50.0f,31.56,21.64, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-50.0f,31.56,21.64,-45.0f,Turn_R};
const static t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400,&ff_gain_out45_L_1400};
const static t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400,&ff_gain_out45_R_1400};

//k= 250
const static t_pid_gain sp_gain_turnIn135_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_1400 = {0.20,0.01,0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,21.48,31.96, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,21.48,31.86,-135.0f,Turn_R};
const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,7.75,8.99, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,7.75,8.99,-135.0f,Turn_R};
const static t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400,&ff_gain_in135_L_1400};
const static t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400,&ff_gain_in135_R_1400};

//220
const static t_pid_gain sp_gain_turnOut135_1400 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_1400 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 42.0f,13.88,39.77, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-42.0f,13.88,39.77,-135.0f,Turn_R};
const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 40.0f,4.10,24.82, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-40.0f,4.10,24.82,-135.0f,Turn_R};
const static t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400,&ff_gain_out135_L_1400};
const static t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400,&ff_gain_out135_R_1400};

const static t_param *const mode_1400[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400
											};

const static t_param *const mode_1400_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400,
												NULL,				NULL
											};
#endif /* CPP_PARAMS_TURN_1400_H_ */
