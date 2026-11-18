/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

// 値の定義は同じディレクトリのturn_1600.cppにあります。

#include "Params/typedef_run_param.h"

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
extern const t_ff_gain ff_gain_long_turn_R_1600;
extern const t_ff_gain ff_gain_long_turn_L_1600;

// The long-90 R/L gains provide the 1600 mm/s yaw-plant baseline. Short V90
// and in45 profiles have dedicated acceleration/deceleration corrections from
// their measured response; the remaining shapes retain the baseline first try.
// See tools/turn_analysis/1600/profile_projection.csv for resulting voltages.
extern const t_ff_gain ff_gain_long_turn_180_R_1600;
extern const t_ff_gain ff_gain_long_turn_180_L_1600;

extern const t_ff_gain ff_gain_v90_R_1600;
extern const t_ff_gain ff_gain_v90_L_1600;
extern const t_ff_gain ff_gain_in45_R_1600;
extern const t_ff_gain ff_gain_in45_L_1600;
extern const t_ff_gain ff_gain_out45_R_1600;
extern const t_ff_gain ff_gain_out45_L_1600;
extern const t_ff_gain ff_gain_in135_R_1600;
extern const t_ff_gain ff_gain_in135_L_1600;
extern const t_ff_gain ff_gain_out135_R_1600;
extern const t_ff_gain ff_gain_out135_L_1600;

// Lstart/Lend are the median ground-track values from the 20260724
// Lstart=Lend=0 marker videos.  The video trajectory was synchronized to the
// logged angular profile.  Long-180 additionally places its apex at 95 mm.
//k = 250
extern const t_pid_gain sp_gain_turn90_1600;
extern const t_pid_gain om_gain_turn90_1600;
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.95,30.5+15.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.95,30.5+15.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.69,34.06, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.69,34.06,-90.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L90_1600_table;
extern const t_turn_param_table slalom_R90_1600_table;
#else
extern const t_turn_param_table slalom_L90_1600_table;
extern const t_turn_param_table slalom_R90_1600_table;
#endif

extern const t_param param_L90_1600;
extern const t_param param_R90_1600;

//k = 250
extern const t_pid_gain sp_gain_turn180_1600;
extern const t_pid_gain om_gain_turn180_1600;
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,8.80,30.10, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,8.80,30.10,-180.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,10.79,32.40, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,10.79,32.40,-180.0f,Turn_R,0.0f};
/*20260725_*/
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,22.68f,26.33f, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,22.68f,26.33f,-180.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L180_1600_table;
extern const t_turn_param_table slalom_R180_1600_table;
#else
extern const t_turn_param_table slalom_L180_1600_table;
extern const t_turn_param_table slalom_R180_1600_table;
#endif
extern const t_param param_L180_1600;
extern const t_param param_R180_1600;

//k = 250
extern const t_pid_gain sp_gain_turnV90_1600;
extern const t_pid_gain om_gain_turnV90_1600;
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.79,24.95-10.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.79,24.95-10.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.51,23.78, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.51,23.78,-90.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_LV90_1600_table;
extern const t_turn_param_table slalom_RV90_1600_table;
#else
extern const t_turn_param_table slalom_LV90_1600_table;
extern const t_turn_param_table slalom_RV90_1600_table;
#endif
extern const t_param param_LV90_1600;
extern const t_param param_RV90_1600;

//k = 250
extern const t_pid_gain sp_gain_turnIn45_1600;
extern const t_pid_gain om_gain_turnIn45_1600;
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.52,39.62+15.0, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.52,39.62+15.0,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,25.41,23.05, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,25.41,23.05,-45.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_inL45_1600_table;
extern const t_turn_param_table slalom_inR45_1600_table;
#else
extern const t_turn_param_table slalom_inL45_1600_table;
extern const t_turn_param_table slalom_inR45_1600_table;
#endif

extern const t_param param_inL45_1600;
extern const t_param param_inR45_1600;

//k = 250
extern const t_pid_gain sp_gain_turnOut45_1600;
extern const t_pid_gain om_gain_turnOut45_1600;
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.06,20.07, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.06,20.07,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.00,20.61, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.00,20.61,-45.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL45_1600_table;
extern const t_turn_param_table slalom_outR45_1600_table;
#else
extern const t_turn_param_table slalom_outL45_1600_table;
extern const t_turn_param_table slalom_outR45_1600_table;
#endif
extern const t_param param_outL45_1600;
extern const t_param param_outR45_1600;

//k = 250,alpha = 1.0
extern const t_pid_gain sp_gain_turnIn135_1600;
extern const t_pid_gain om_gain_turnIn135_1600;//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,8.23+5.0,14.76+10.0, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,8.23+5.0,14.76+10.0,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.0f,11.60,25.47, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.0f,11.60,25.47,-135.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_inL135_1600_table;
extern const t_turn_param_table slalom_inR135_1600_table;
#else
extern const t_turn_param_table slalom_inL135_1600_table;
extern const t_turn_param_table slalom_inR135_1600_table;
#endif
extern const t_param param_inL135_1600;
extern const t_param param_inR135_1600;

//k = 250
extern const t_pid_gain sp_gain_turnOut135_1600;
extern const t_pid_gain om_gain_turnOut135_1600;
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,8.09,37.73, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,8.09,37.73,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 39.3f,12.47,40.99, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-39.3f,12.47,40.99,-135.0f,Turn_R,0.0f};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL135_1600_table;
extern const t_turn_param_table slalom_outR135_1600_table;
#else
extern const t_turn_param_table slalom_outL135_1600_table;
extern const t_turn_param_table slalom_outR135_1600_table;
#endif
extern const t_param param_outL135_1600;
extern const t_param param_outR135_1600;

extern const t_param *const mode_1600[17];



extern const t_param *const mode_1600_acc_v1[19];

extern const t_param *const mode_1600_acc_v2[19];



#endif /* CPP_PARAMS_TURN_1600_H_ */
