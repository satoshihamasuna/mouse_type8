/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

#include "typedef_run_param.h"

// Long-90 OM feedforward adjusted from the 20260720 voltage-to-response
// analysis. OM velocity and signed bias retain their verified values because a
// single fixed-speed profile cannot identify them independently. Acceleration,
// deceleration, and jerk also account for turn-body tracking and post-turn
// angular-velocity ringing instead of fitting PID compensation alone.
// R/L-specific SP turn feedforward was identified from the 20260722 long-180
// voltage response.  The directional baseline is shared by the turn shapes.
// Long-180 itself uses a second 50% iterative correction from the repeated
// FF-on logs; this avoids projecting the stronger correction into unmeasured
// short shapes and keeps the long-180 voltage below the common 1.0 V clamp.
const static t_ff_gain ff_gain_long_turn_R_1600 = {
	0.9030126f, 0.08183161f, 0.2379401f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_long_turn_L_1600 = {
	0.9030126f, 0.08183161f, 0.2400677f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};

// The long-90 R/L gains provide the 1600 mm/s yaw-plant baseline. Short V90
// and in45 profiles have dedicated acceleration/deceleration corrections from
// their measured response; the remaining shapes retain the baseline first try.
// See tools/turn_analysis/1600/profile_projection.csv for resulting voltages.
const static t_ff_gain ff_gain_long_turn_180_R_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00024186f, 0.00008612f
};
const static t_ff_gain ff_gain_long_turn_180_L_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00023491f, 0.00010562f
};

const static t_ff_gain ff_gain_v90_R_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00093f, 0.00090f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_v90_L_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00099f, 0.00091f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const static t_ff_gain ff_gain_in45_R_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00098f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_in45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00099f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const static t_ff_gain ff_gain_out45_R_1600 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_out45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const static t_ff_gain ff_gain_in135_R_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_in135_L_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const static t_ff_gain ff_gain_out135_R_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const static t_ff_gain ff_gain_out135_L_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};

// Lstart/Lend are the median ground-track values from the 20260724
// Lstart=Lend=0 marker videos.  The video trajectory was synchronized to the
// logged angular profile.  Long-180 additionally places its apex at 95 mm.
//k = 250
const static t_pid_gain sp_gain_turn90_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.95,30.5+15.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.95,30.5+15.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.69,34.06, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.69,34.06,-90.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,15.97f,27.58f, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,15.97f,27.58f,-90.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,18.42f,26.31f, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,18.42f,26.31f,-90.0f,Turn_R,0.0f};
#endif

const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_L_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_R_1600};

//k = 250
const static t_pid_gain sp_gain_turn180_1600 ={2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_1600 ={0.20,0.012,0.0};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,8.80,30.10, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,8.80,30.10,-180.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,10.79,32.40, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,10.79,32.40,-180.0f,Turn_R,0.0f};
/*20260725_*/
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,22.68f,26.33f, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,22.68f,26.33f,-180.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,13.12f,27.38f, 180.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,13.12f,27.38f,-180.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,13.12,27.38, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,13.12,27.38,-180.0f,Turn_R};
#endif
const static t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_L_1600};
const static t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnV90_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.79,24.95-10.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.79,24.95-10.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.51,23.78, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.51,23.78,-90.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,6.87f,21.41f, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,6.87f,21.41f,-90.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,6.49f,20.83f, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,6.49f,20.83f,-90.0f,Turn_R,0.0f};
#endif
const static t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_L_1600};
const static t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnIn45_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_1600 = {0.120,0.004,0.0};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.52,39.62+15.0, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.52,39.62+15.0,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,25.41,23.05, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,25.41,23.05,-45.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,8.81f,40.24f, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,8.81f,40.24f,-45.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,6.46f,39.86f, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,6.46f,39.86f,-45.0f,Turn_R,0.0f};
#endif

const static t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_L_1600};
const static t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnOut45_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.06,20.07, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.06,20.07,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.00,20.61, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.00,20.61,-45.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,24.62f,18.06f, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,24.62f,18.06f,-45.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.74f,20.46f, 45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.74f,20.46f,-45.0f,Turn_R,0.0f};
#endif
const static t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_L_1600};
const static t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_R_1600};

//k = 250,alpha = 1.0
const static t_pid_gain sp_gain_turnIn135_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_1600 = {0.2,0.01,0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,8.23+5.0,14.76+10.0, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,8.23+5.0,14.76+10.0,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.0f,11.60,25.47, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.0f,11.60,25.47,-135.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,15.09f,26.35f, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,15.09f,26.35f,-135.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,17.43f,20.56f, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,17.43f,20.56f,-135.0f,Turn_R,0.0f};
#endif
const static t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_L_1600};
const static t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_R_1600};

//k = 250
const static t_pid_gain sp_gain_turnOut135_1600 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,8.09,37.73, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,8.09,37.73,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 39.3f,12.47,40.99, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-39.3f,12.47,40.99,-135.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,12.10f,36.93f, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,12.10f,36.93f,-135.0f,Turn_R,0.0f};
#else
const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,11.13f,36.50f, 135.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,11.13f,36.50f,-135.0f,Turn_R,0.0f};
#endif
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
