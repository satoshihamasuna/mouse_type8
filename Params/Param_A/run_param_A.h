/*
 * run_param_A.h
 *
 *  Created on: Nov 2, 2025
 *      Author: sato1
 */

#ifndef RUN_PARAM_A_H_
#define RUN_PARAM_A_H_

// 値の定義は同じディレクトリのrun_param_A.cppにあります。

// 0: currently adopted turn lengths
// 1: Lstart/Lend calculated from the pink (turn-LED-on) video interval
#define PARAM_A_USE_PINK_LED_TURN_LENGTHS 0

#include "Params/Param_A/turn_1000/turn_1000.h"
#include "Params/Param_A/turn_1200/turn_1200.h"
#include "Params/Param_A/turn_1400/turn_1400.h"
#include "Params/Param_A/turn_1500/turn_1500.h"
#include "Params/Param_A/turn_1600/turn_1600.h"
#include "Params/Param_A/turn_1800/turn_1800.h"
#include "Params/Param_A/turn_2000/turn_2000.h"
#include "Params/Param_A/turn_300/turn_300.h"
#include "Params/Param_A/turn_500/turn_500.h"
#include "Params/Param_A/turn_700/turn_700.h"
#include "Component/Inc/controller.h"
#include "Module/Inc/vehicle.h"
#include "Params/typedef_run_param.h"

extern const t_pid_gain basic_sp_gain;//{10.0,0.05,0.00};////
extern const t_pid_gain basic_om_gain;//;{0.40f, 0.05f, 0.00f};//

extern const t_pid_gain search_sp_gain;//{10.0,0.05,0.00};////
extern const t_pid_gain search_om_gain;//{0.40f, 0.05f, 0.00f};//

// Feedback gains used exclusively by pivot turns.
extern const t_pid_gain sp_gain_pivot_turn;
extern const t_pid_gain om_gain_pivot_turn;

// Pivot-turn feedforward identified from the 2026-07-15 right/left logs.
// Keep the physical angular-velocity coefficient fixed because it is strongly
// correlated with the signed bias in a single-speed pivot profile.
extern const t_ff_gain ff_gain_pivot_turn_R;
extern const t_ff_gain ff_gain_pivot_turn_L;

#include "Params/Param_A/turn_search/turn_search.h"


// Straight motion parameters for each velocity setting. The acceleration is
// determined by the velocity and the desired motion profile.

// Common non-suction SP feedforward for straight parameters at 700 mm/s and
// below. Search turns use the same SP coefficients.

extern const t_ff_gain ff_gain_straight_non_suction;

extern const t_ff_gain ff_gain_straight_suction;

// 最短plain500の速度条件。値はrun_param_A.cpp。直進と斜めは個別に調整する。
extern const t_speed_profile straight_speed_plain500;
extern const t_speed_profile diagonal_speed_plain500;

// 直進/斜めの全速度設定。単一範囲・複数範囲とも同じ型で参照する。
extern const t_speed_profile straight_speed_300_v0;
extern const t_speed_profile straight_speed_300_v1;
extern const t_speed_profile straight_speed_700_v0;
extern const t_speed_profile straight_speed_1000_v0;
extern const t_speed_profile straight_speed_1000_v1;
extern const t_speed_profile straight_speed_1200_v0;
extern const t_speed_profile straight_speed_1200_v1;
extern const t_speed_profile straight_speed_1400_v0;
extern const t_speed_profile straight_speed_1400_v1;
extern const t_speed_profile straight_speed_1400_v2;
extern const t_speed_profile straight_speed_1400_v3;
extern const t_speed_profile straight_speed_1500_v0;
extern const t_speed_profile straight_speed_1500_v1;
extern const t_speed_profile straight_speed_1500_v2;
extern const t_speed_profile straight_speed_1600_v1;
extern const t_speed_profile straight_speed_1600_v2;
extern const t_speed_profile straight_speed_1600_v3;
extern const t_speed_profile straight_speed_1800_v1;
extern const t_speed_profile straight_speed_2000_v1;
extern const t_speed_profile diagonal_speed_300_v0;
extern const t_speed_profile diagonal_speed_300_v1;
extern const t_speed_profile diagonal_speed_700_v0;
extern const t_speed_profile diagonal_speed_1000_v0;
extern const t_speed_profile diagonal_speed_1000_v1;
extern const t_speed_profile diagonal_speed_1200_v0;
extern const t_speed_profile diagonal_speed_1200_v1;
extern const t_speed_profile diagonal_speed_1400_v0;
extern const t_speed_profile diagonal_speed_1400_v1;
extern const t_speed_profile diagonal_speed_1400_v2;
extern const t_speed_profile diagonal_speed_1500_v0;
extern const t_speed_profile diagonal_speed_1500_v1;
extern const t_speed_profile diagonal_speed_1500_v2;
extern const t_speed_profile diagonal_speed_1600_v1;
extern const t_speed_profile diagonal_speed_1600_v2;
extern const t_speed_profile diagonal_speed_1800_v1;
extern const t_speed_profile diagonal_speed_2000_v1;

extern const t_pid_gain sp_gain_280;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_280;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_280;
extern const t_straight_param st_param_280;

extern const t_pid_gain sp_gain_300;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_300;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_300;
extern const t_straight_param st_param_300;

extern const t_pid_gain sp_gain_320;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_320;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_320;
extern const t_straight_param st_param_320;

extern const t_pid_gain sp_gain_350;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_350;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_350;
extern const t_straight_param st_param_350;

extern const t_pid_gain sp_gain_370;
extern const t_pid_gain om_gain_370;
extern const t_velo_param param_370;
extern const t_straight_param st_param_370;


extern const t_pid_gain sp_gain_400;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_400;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_400;
extern const t_straight_param st_param_400;

extern const t_pid_gain sp_gain_450;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_450;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_450;
extern const t_straight_param st_param_450;

extern const t_pid_gain sp_gain_500;//{10.0,0.05,0.00};//
extern const t_pid_gain om_gain_500;//{0.40f, 0.05f, 0.00f};//
extern const t_velo_param param_500;
extern const t_straight_param st_param_500;

extern const t_pid_gain sp_gain_600;
extern const t_pid_gain om_gain_600;
extern const t_velo_param param_600;
extern const t_straight_param st_param_600;

extern const t_pid_gain sp_gain_700;
extern const t_pid_gain om_gain_700;
extern const t_velo_param param_700;
extern const t_straight_param st_param_700;


extern const t_pid_gain sp_gain_1000;
extern const t_pid_gain om_gain_1000;
extern const t_velo_param param_1000;
extern const t_straight_param st_param_1000;

extern const t_pid_gain sp_gain_1050;
extern const t_pid_gain om_gain_1050;
extern const t_velo_param param_1050;
extern const t_straight_param st_param_1050;

extern const t_pid_gain sp_gain_1100;
extern const t_pid_gain om_gain_1100;
extern const t_velo_param param_1100;
extern const t_straight_param st_param_1100;

extern const t_pid_gain sp_gain_1200;
extern const t_pid_gain om_gain_1200;
extern const t_velo_param param_1200;
extern const t_straight_param st_param_1200;

extern const t_pid_gain sp_gain_1300;
extern const t_pid_gain om_gain_1300;
extern const t_velo_param param_1300;
extern const t_straight_param st_param_1300;

extern const t_pid_gain sp_gain_1400;
extern const t_pid_gain om_gain_1400;
extern const t_velo_param param_1400;
extern const t_velo_param param_1400_acc2G;
extern const t_velo_param param_1400_acc3G;
extern const t_straight_param st_param_1400;
extern const t_straight_param st_param_1400_acc2G;
extern const t_straight_param st_param_1400_acc3G;

extern const t_pid_gain sp_gain_1500;
extern const t_pid_gain om_gain_1500;
extern const t_velo_param param_1500;
extern const t_velo_param param_1500_acc2G;
extern const t_velo_param param_1500_acc3G;
extern const t_straight_param st_param_1500;
extern const t_straight_param st_param_1500_acc2G;
extern const t_straight_param st_param_1500_acc3G;

extern const t_pid_gain sp_gain_1600;
extern const t_pid_gain om_gain_1600;
extern const t_velo_param param_1600;
extern const t_velo_param param_1600_acc2G;
extern const t_velo_param param_1600_acc3G;
extern const t_straight_param st_param_1600;
extern const t_straight_param st_param_1600_acc2G;
extern const t_straight_param st_param_1600_acc3G;

extern const t_pid_gain sp_gain_1700;
extern const t_pid_gain om_gain_1700;
extern const t_velo_param param_1700;
extern const t_velo_param param_1700_acc2G;
extern const t_velo_param param_1700_acc3G;
extern const t_straight_param st_param_1700;
extern const t_straight_param st_param_1700_acc2G;
extern const t_straight_param st_param_1700_acc3G;

extern const t_pid_gain sp_gain_1800;
extern const t_pid_gain om_gain_1800;
extern const t_velo_param param_1800;
extern const t_velo_param param_1800_acc2G;
extern const t_velo_param param_1800_acc3G;
extern const t_straight_param st_param_1800;
extern const t_straight_param st_param_1800_acc2G;
extern const t_straight_param st_param_1800_acc3G;


extern const t_pid_gain sp_gain_1900;
extern const t_pid_gain om_gain_1900;
extern const t_velo_param param_1900;
extern const t_velo_param param_1900_acc2G;
extern const t_velo_param param_1900_acc3G;
extern const t_straight_param st_param_1900;
extern const t_straight_param st_param_1900_acc2G;
extern const t_straight_param st_param_1900_acc3G;

extern const t_pid_gain sp_gain_2000;
extern const t_pid_gain om_gain_2000;
extern const t_velo_param param_2000;
extern const t_velo_param param_2000_acc2G;
extern const t_velo_param param_2000_acc3G;
extern const t_straight_param st_param_2000;
extern const t_straight_param st_param_2000_acc2G;
extern const t_straight_param st_param_2000_acc3G;


extern const t_pid_gain sp_gain_2100;
extern const t_pid_gain om_gain_2100;
extern const t_velo_param param_2100;
extern const t_velo_param param_2100_acc2G;
extern const t_velo_param param_2100_acc3G;
extern const t_straight_param st_param_2100;
extern const t_straight_param st_param_2100_acc2G;
extern const t_straight_param st_param_2100_acc3G;

extern const t_pid_gain sp_gain_2200;
extern const t_pid_gain om_gain_2200;
extern const t_velo_param param_2200;
extern const t_velo_param param_2200_acc2G;
extern const t_velo_param param_2200_acc3G;
extern const t_straight_param st_param_2200;
extern const t_straight_param st_param_2200_acc2G;
extern const t_straight_param st_param_2200_acc3G;

extern const t_pid_gain sp_gain_2300;
extern const t_pid_gain om_gain_2300;
extern const t_velo_param param_2300;
extern const t_velo_param param_2300_acc2G;
extern const t_velo_param param_2300_acc3G;
extern const t_straight_param st_param_2300;
extern const t_straight_param st_param_2300_acc2G;
extern const t_straight_param st_param_2300_acc3G;

extern const t_pid_gain sp_gain_2400;
extern const t_pid_gain om_gain_2400;
extern const t_velo_param param_2400;
extern const t_velo_param param_2400_acc2G;
extern const t_velo_param param_2400_acc3G;
extern const t_straight_param st_param_2400;
extern const t_straight_param st_param_2400_acc2G;
extern const t_straight_param st_param_2400_acc3G;

extern const t_pid_gain sp_gain_2500;
extern const t_pid_gain om_gain_2500;
extern const t_velo_param param_2500;
extern const t_velo_param param_2500_acc2G;
extern const t_velo_param param_2500_acc3G;
extern const t_straight_param st_param_2500;
extern const t_straight_param st_param_2500_acc2G;
extern const t_straight_param st_param_2500_acc3G;

extern const t_pid_gain sp_gain_2600;
extern const t_pid_gain om_gain_2600;
extern const t_velo_param param_2600;
extern const t_velo_param param_2600_acc2G;
extern const t_velo_param param_2600_acc3G;
extern const t_straight_param st_param_2600;
extern const t_straight_param st_param_2600_acc2G;
extern const t_straight_param st_param_2600_acc3G;

extern const t_pid_gain sp_gain_2700;
extern const t_pid_gain om_gain_2700;
extern const t_velo_param param_2700;
extern const t_velo_param param_2700_acc2G;
extern const t_velo_param param_2700_acc3G;
extern const t_straight_param st_param_2700;
extern const t_straight_param st_param_2700_acc2G;
extern const t_straight_param st_param_2700_acc3G;


extern const t_pid_gain sp_gain_2800;
extern const t_pid_gain om_gain_2800;
extern const t_velo_param param_2800;
extern const t_velo_param param_2800_acc2G;
extern const t_velo_param param_2800_acc3G;
extern const t_straight_param st_param_2800;
extern const t_straight_param st_param_2800_acc2G;
extern const t_straight_param st_param_2800_acc3G;


extern const t_pid_gain sp_gain_2900;
extern const t_pid_gain om_gain_2900;
extern const t_velo_param param_2900;
extern const t_velo_param param_2900_acc2G;
extern const t_velo_param param_2900_acc3G;
extern const t_straight_param st_param_2900;
extern const t_straight_param st_param_2900_acc2G;
extern const t_straight_param st_param_2900_acc3G;

extern const t_pid_gain sp_gain_3000;
extern const t_pid_gain om_gain_3000;
extern const t_velo_param param_3000;
extern const t_velo_param param_3000_acc2G;
extern const t_velo_param param_3000_acc3G;
extern const t_straight_param st_param_3000;
extern const t_straight_param st_param_3000_acc2G;
extern const t_straight_param st_param_3000_acc3G;

extern const t_pid_gain sp_gain_3200;
extern const t_pid_gain om_gain_3200;
extern const t_velo_param param_3200;
extern const t_velo_param param_3200_acc2G;
extern const t_velo_param param_3200_acc3G;
extern const t_straight_param st_param_3200;
extern const t_straight_param st_param_3200_acc2G;
extern const t_straight_param st_param_3200_acc3G;

extern const t_pid_gain sp_gain_3400;
extern const t_pid_gain om_gain_3400;
extern const t_velo_param param_3400;
extern const t_velo_param param_3400_acc2G;
extern const t_velo_param param_3400_acc3G;
extern const t_straight_param st_param_3400;
extern const t_straight_param st_param_3400_acc2G;
extern const t_straight_param st_param_3400_acc3G;

extern const t_pid_gain sp_gain_3600;
extern const t_pid_gain om_gain_3600;
extern const t_velo_param param_3600;
extern const t_velo_param param_3600_acc2G;
extern const t_velo_param param_3600_acc3G;
extern const t_straight_param st_param_3600;
extern const t_straight_param st_param_3600_acc2G;
extern const t_straight_param st_param_3600_acc3G;

extern const t_pid_gain sp_gain_3800;
extern const t_pid_gain om_gain_3800;
extern const t_velo_param param_3800;
extern const t_velo_param param_3800_acc2G;
extern const t_velo_param param_3800_acc3G;
extern const t_straight_param st_param_3800;
extern const t_straight_param st_param_3800_acc2G;
extern const t_straight_param st_param_3800_acc3G;

extern const t_pid_gain sp_gain_4000;
extern const t_pid_gain om_gain_4000;
extern const t_velo_param param_4000;
extern const t_velo_param param_4000_acc2G;
extern const t_velo_param param_4000_acc3G;
extern const t_straight_param st_param_4000;
extern const t_straight_param st_param_4000_acc2G;
extern const t_straight_param st_param_4000_acc3G;

extern const t_pid_gain sp_gain_4200;
extern const t_pid_gain om_gain_4200;
extern const t_velo_param param_4200;
extern const t_velo_param param_4200_acc2G;
extern const t_velo_param param_4200_acc3G;
extern const t_straight_param st_param_4200;
extern const t_straight_param st_param_4200_acc2G;
extern const t_straight_param st_param_4200_acc3G;

extern const t_pid_gain sp_gain_4400;
extern const t_pid_gain om_gain_4400;
extern const t_velo_param param_4400;
extern const t_velo_param param_4400_acc2G;
extern const t_velo_param param_4400_acc3G;
//const static t_velo_param param_4600_acc5G = {4.4f,50.0f};
extern const t_straight_param st_param_4400;
extern const t_straight_param st_param_4400_acc2G;
extern const t_straight_param st_param_4400_acc3G;

extern const t_pid_gain sp_gain_4600;
extern const t_pid_gain om_gain_4600;
extern const t_velo_param param_4600;
extern const t_velo_param param_4600_acc2G;
extern const t_velo_param param_4600_acc3G;
//const static t_velo_param param_4600_acc5G = {4.6f,50.0f};
extern const t_straight_param st_param_4600;
extern const t_straight_param st_param_4600_acc2G;
extern const t_straight_param st_param_4600_acc3G;

extern const t_velo_param param_1400_acc5G;
extern const t_straight_param st_param_1400_acc5G;

extern const t_velo_param param_1500_acc5G;
extern const t_straight_param st_param_1500_acc5G;

extern const t_velo_param param_1600_acc5G;
extern const t_straight_param st_param_1600_acc5G;

extern const t_velo_param param_1700_acc5G;
extern const t_straight_param st_param_1700_acc5G;

extern const t_velo_param param_1800_acc5G;
extern const t_straight_param st_param_1800_acc5G;

extern const t_velo_param param_1900_acc5G;
extern const t_straight_param st_param_1900_acc5G;

extern const t_velo_param param_2000_acc5G;
extern const t_straight_param st_param_2000_acc5G;

extern const t_velo_param param_2100_acc5G;
extern const t_straight_param st_param_2100_acc5G;

extern const t_velo_param param_2200_acc5G;
extern const t_straight_param st_param_2200_acc5G;

extern const t_velo_param param_2300_acc5G;
extern const t_straight_param st_param_2300_acc5G;

extern const t_velo_param param_2400_acc5G;
extern const t_straight_param st_param_2400_acc5G;

extern const t_velo_param param_2500_acc5G;
extern const t_straight_param st_param_2500_acc5G;

extern const t_velo_param param_2600_acc5G;
extern const t_straight_param st_param_2600_acc5G;

extern const t_velo_param param_2700_acc5G;
extern const t_straight_param st_param_2700_acc5G;

extern const t_velo_param param_2800_acc5G;
extern const t_straight_param st_param_2800_acc5G;

extern const t_velo_param param_2900_acc5G;
extern const t_straight_param st_param_2900_acc5G;

extern const t_velo_param param_3000_acc5G;
extern const t_straight_param st_param_3000_acc5G;

extern const t_velo_param param_3200_acc5G;
extern const t_straight_param st_param_3200_acc5G;

extern const t_velo_param param_3400_acc5G;
extern const t_straight_param st_param_3400_acc5G;

extern const t_velo_param param_3600_acc5G;
extern const t_straight_param st_param_3600_acc5G;

extern const t_velo_param param_3800_acc5G;
extern const t_straight_param st_param_3800_acc5G;

extern const t_velo_param param_4000_acc5G;
extern const t_straight_param st_param_4000_acc5G;

extern const t_velo_param param_4200_acc5G;
extern const t_straight_param st_param_4200_acc5G;

extern const t_velo_param param_4400_acc5G;
extern const t_straight_param st_param_4400_acc5G;

extern const t_velo_param param_4600_acc5G;
extern const t_straight_param st_param_4600_acc5G;





extern const t_straight_param *const st_mode_300_v0[1];
extern const t_straight_param *const st_mode_300_v1[2];
extern const t_straight_param *const st_mode_500_v0[3];
extern const t_straight_param *const st_mode_700_v0[1];
extern const t_straight_param *const st_mode_1000_v0[1];
extern const t_straight_param *const st_mode_1000_v1[7];
extern const t_straight_param *const st_mode_1200_v0[4];
extern const t_straight_param *const st_mode_1200_v1[12];

extern const t_straight_param *const st_mode_1400_v0[9];

extern const t_straight_param *const st_mode_1400_v1[11];
extern const t_straight_param *const st_mode_1400_v2[17];
extern const t_straight_param *const st_mode_1400_v3[17];



extern const t_straight_param *const st_mode_1500_v0[9];
extern const t_straight_param *const st_mode_1500_v1[9];

extern const t_straight_param *const st_mode_1500_v2[17];


extern const t_straight_param *const st_mode_1600_v1[16];

extern const t_straight_param *const st_mode_1600_v2[16];

extern const t_straight_param *const st_mode_1600_v3[16];
extern const t_straight_param *const st_mode_1800_v1[15];

extern const t_straight_param *const st_mode_2000_v1[14];

extern const t_straight_param *const di_mode_300_v0[1];
extern const t_straight_param *const di_mode_300_v1[2];
extern const t_straight_param *const di_mode_500_v0[3];
extern const t_straight_param *const di_mode_700_v0[1];
extern const t_straight_param *const di_mode_1000_v0[1];
extern const t_straight_param *const di_mode_1000_v1[7];
extern const t_straight_param *const di_mode_1200_v0[4];
extern const t_straight_param *const di_mode_1200_v1[12];
extern const t_straight_param *const di_mode_1400_v0[4];
extern const t_straight_param *const di_mode_1400_v1[9];
extern const t_straight_param *const di_mode_1400_v2[17];

extern const t_straight_param *const di_mode_1500_v0[9];
extern const t_straight_param *const di_mode_1500_v1[9];
extern const t_straight_param *const di_mode_1500_v2[17];

extern const t_straight_param *const di_mode_1600_v1[16];

extern const t_straight_param *const di_mode_1600_v2[16];


extern const t_straight_param *const di_mode_1800_v1[15];

extern const t_straight_param *const di_mode_2000_v1[14];



extern const t_pid_gain sp_gain_dummy;
extern const t_pid_gain om_gain_dummy;
extern const t_turn_param_table slalom_dummy;
extern const t_param param_dummy;


extern const t_param *const *const acc_mode_1400[2];
extern const t_param *const *const acc_mode_1600_v1[2];
extern const t_param *const *const acc_mode_1600_v2[2];
extern const t_param *const *const acc_mode_1600_v3[3];
extern const t_param *const *const acc_mode_1800_v1[2];



#endif /* RUN_PARAM_A_H_ */
