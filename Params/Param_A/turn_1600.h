/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

#include "typedef_run_param.h"

// Second-pass identification from the suction-run verification logs captured
// at and after 20260716_034241. SP velocity is retained because a single fixed
// speed cannot separate it reliably from bias; SP acceleration/bias are
// refitted. Both free angular fits are negative, so OM velocity is constrained
// to zero before refitting angular acceleration and signed bias per direction.
const static t_ff_gain ff_gain_long_turn_R_1600 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.002f, 0.00096f, 0.00098f, 0.2972200f
};
const static t_ff_gain ff_gain_long_turn_L_1600 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.002f, 0.00085f, 0.00106f, 0.43f
};

// Second-pass long-180 identification from the suction-run verification logs
// captured after 20260716_035548. The incomplete-active-phase 040457 left log
// is excluded. SP velocity is retained from the identified 1600 mm/s suction
// profile; both angular velocity fits remain positive and consistent.
const static t_ff_gain ff_gain_long_turn_180_R_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.007355744f, 0.0009893826f, 0.0009893826f, 0.1964514f
};
const static t_ff_gain ff_gain_long_turn_180_L_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.008186232f, 0.0009810843f, 0.0009810843f, 0.1963813f
};

// Dedicated suction-run feedforward identified from the non-long-turn logs
// captured at and after 20260716_023755. The already identified suction SP
// velocity coefficient is retained because these are single-speed logs.
// Negative free OM velocity fits are constrained to zero and then refitted.
const static t_ff_gain ff_gain_v90_R_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0f, 0.00085f, 0.00065f, 0.250f
};
const static t_ff_gain ff_gain_v90_L_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0f, 0.00091f, 0.00102f, 0.40f
};
// Right in45 FF verified with the three 1600 mm/s logs captured at 221653,
// 221732, and 221815. Their fitted angular plant has a 3 ms input delay.
// Acceleration/deceleration gains prioritize peak-overshoot suppression with
// the 0.20/0.005 PI gains and battery-voltage saturation included.
const static t_ff_gain ff_gain_in45_R_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.000825f, 0.000625f, 0.0f
};
const static t_ff_gain ff_gain_in45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.00092f, 0.00070f, 0.04f
};
const static t_ff_gain ff_gain_out45_R_1600 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0f, 0.000825f, 0.000625f, 0.0f
};
const static t_ff_gain ff_gain_out45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0f, 0.00092f, 0.00070f, 0.04f
};
const static t_ff_gain ff_gain_in135_R_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.004f, 0.001069892f, 0.001069892f, 0.3761218f
};
const static t_ff_gain ff_gain_in135_L_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.004f, 0.00086f, 0.00113f, 0.485f
};
const static t_ff_gain ff_gain_out135_R_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.004f, 0.001069892f, 0.001069892f, 0.3761218f
};
const static t_ff_gain ff_gain_out135_L_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.004f, 0.00086f, 0.00113f, 0.485f
};

//k = 250
const static t_pid_gain sp_gain_turn90_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.50f,19.56,38.43, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.50f,19.56,38.43,-90.0f,Turn_R};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.95,30.5, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.95,30.5,-90.0f,Turn_R,0.0f};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_L_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_R_1600};

//k = 250
const static t_pid_gain sp_gain_turn180_1600 ={2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_1600 ={0.20,0.012,0.0};
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,8.80,30.10, 180.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,8.80,30.10,-180.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f,  45.0f,(15.10),(34.72), 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f, -45.0f,(15.10),(34.72),-180.0f,Turn_R};
const static t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_L_1600};
const static t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnV90_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,9.50,29.66, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,9.50,29.66,-90.0f,Turn_R};
const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.79,24.95-10.0, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.79,24.95-10.0,-90.0f,Turn_R,0.0f};
const static t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_L_1600};
const static t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnIn45_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_1600 = {0.120,0.004,0.0};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 53.0f,9.84,41.94, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.52,39.62, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.52,39.62,-45.0f,Turn_R,0.0f};
const static t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_L_1600};
const static t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnOut45_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 55.0f,18.235,20.24, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.06,20.07, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.06,20.07,-45.0f,Turn_R,0.0f};
const static t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_L_1600};
const static t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_R_1600};

//k = 250,alpha = 1.0
const static t_pid_gain sp_gain_turnIn135_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_1600 = {0.2,0.01,0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,13.17,26.34, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,13.17,26.34,-135.0f,Turn_R};
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,8.23,14.76, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,8.23,14.76,-135.0f,Turn_R,0.0f};
const static t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_L_1600};
const static t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnOut135_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 42.0f,11.85,40.11, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,8.09,37.73, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,8.09,37.73,-135.0f,Turn_R,0.0f};
const static t_param param_outL135_1600 = {&slalom_outL135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600,&ff_gain_out135_L_1600};
const static t_param param_outR135_1600 = {&slalom_outR135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600,&ff_gain_out135_R_1600};

const static t_param *const mode_1600[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												&param_inR45_1600,	&param_inL45_1600,
												&param_outR45_1600,	&param_outL45_1600,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												&param_RV90_1600,	&param_LV90_1600
											};



const static t_param *const mode_1600_acc_v1[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												NULL,					NULL	,
												NULL,					NULL	,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												NULL,					NULL	,
												NULL,					NULL
											};

const static t_param *const mode_1600_acc_v2[] = 	{	NULL,					NULL,			NULL,
													&param_R90_1600,		&param_L90_1600,
													&param_R180_1600,	&param_L180_1600,
													&param_inR45_1600,	&param_inL45_1600,
													&param_outR45_1600,	&param_outL45_1600,
													&param_inR135_1600,	&param_inL135_1600,
													&param_outR135_1600,	&param_outL135_1600,
													&param_RV90_1600,	&param_LV90_1600,
													NULL,					NULL
											};



#endif /* CPP_PARAMS_TURN_1600_H_ */
