/*
 * run_param_A.h
 *
 *  Created on: Nov 2, 2025
 *      Author: sato1
 */

#ifndef RUN_PARAM_A_H_
#define RUN_PARAM_A_H_

#include <Param_A/turn_1000.h>
#include <Param_A/turn_1200.h>
#include <Param_A/turn_1400.h>
#include <Param_A/turn_1500.h>
#include <Param_A/turn_1600.h>
#include <Param_A/turn_1800.h>
#include <Param_A/turn_2000.h>
#include <Param_A/turn_300.h>
#include <Param_A/turn_500.h>
#include <Param_A/turn_700.h>
#include "../Component/Inc/controller.h"
#include "../Module/Inc/vehicle.h"
#include "typedef_run_param.h"

const static t_pid_gain basic_sp_gain = {2.0,0.04,0.00};//{10.0,0.05,0.00};////
const static t_pid_gain basic_om_gain = {0.08f,0.003f,0.00f};//;{0.40f, 0.05f, 0.00f};//

const static t_pid_gain search_sp_gain = {2.0,0.04,0.00};//{10.0,0.05,0.00};////
const static t_pid_gain search_om_gain = {0.1f,0.003f,0.00f};//{0.40f, 0.05f, 0.00f};//

// Feedback gains used exclusively by pivot turns.
const static t_pid_gain sp_gain_pivot_turn = {2.0f, 0.04f, 0.0f};
const static t_pid_gain om_gain_pivot_turn = {0.1f, 0.005f, 0.0f};

// Pivot-turn feedforward identified from the 2026-07-15 right/left logs.
// Keep the physical angular-velocity coefficient fixed because it is strongly
// correlated with the signed bias in a single-speed pivot profile.
const static t_ff_gain ff_gain_pivot_turn_R = {
	FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, 0.00602f, 0.001043436f, 0.001043436f, 0.3704708f
};
const static t_ff_gain ff_gain_pivot_turn_L = {
	FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, 0.00602f, 0.001002870f, 0.001002870f, 0.3616070f
};

const static t_pid_gain sp_gain_search_turn =  {2.0,0.04,0.00};//{10.0,0.05,0.00};////{2.0,0.04};
const static t_pid_gain om_gain_search_turn =  {0.1f,0.005f,0.00f};//{0.60f, 0.05f, 0.000f};//{0.50f, 0.0005f, 0.001f};

// Search-turn feedforward is kept per speed and direction so results obtained
// with myshell_debug.py can be applied independently after identification.
// Search-turn OM feedforward uses a conservative 25% learning update from
// measured closed-loop voltage.  The deceleration fit weights the final 35%
// of the angular-speed profile to reduce exit angular-velocity residue.  Jerk
// starts at 6.25% of the identified value because it is newly enabled here.
// Common non-suction SP feedforward identified from the 300, 500 and
// 700 mm/s straight logs and shared by search turns.  The final two values in
// each search-turn gain are the turn-induced common-mode correction:
// K_sp_turn_alpha * sign(target omega) * target alpha
//     + K_sp_turn_omega2 * target omega^2.
// The values below are a conservative 50% learning update from the voltage-
// response model identified with the 2026-07-21/22 baseline and trial logs.
const static float FF_SEARCH_SP_VELO_COEF = 0.4379f;
const static float FF_SEARCH_SP_ACCEL_COEF = 0.1000f;
const static float FF_SEARCH_SP_BIAS_COEF = 0.3329f;
const static t_ff_gain ff_gain_search_turn_R_400 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0064590f, 0.0015171f, 0.0009990f, 0.0188340f, 0.00000032f, -0.00056554f, 0.00007881f};
const static t_ff_gain ff_gain_search_turn_L_400 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0072780f, 0.0014828f, 0.0010880f, 0.0076013f, 0.00000032f, -0.00012914f, 0.00005493f};
const static t_ff_gain ff_gain_search_turn_R_370 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0053600f, 0.0015777f, 0.0008080f, 0.0443326f, 0.00000032f, -0.00053817f, 0.00020249f};
const static t_ff_gain ff_gain_search_turn_L_370 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0065180f, 0.0014980f, 0.0010350f, 0.0148286f, 0.00000032f, -0.00016902f, 0.00012027f};
const static t_ff_gain ff_gain_search_turn_R_350 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0049760f, 0.0015374f, 0.0009230f, 0.0340973f, 0.00000032f, -0.00045500f, 0.00021685f};
const static t_ff_gain ff_gain_search_turn_L_350 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0060030f, 0.0014087f, 0.0010080f, 0.0199057f, 0.00000032f, -0.00018576f, 0.00013722f};
const static t_ff_gain ff_gain_search_turn_R_320 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0048070f, 0.0014096f, 0.0007200f, 0.0318284f, 0.00000032f, -0.00036311f, 0.00021241f};
const static t_ff_gain ff_gain_search_turn_L_320 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0056160f, 0.0012020f, 0.0009570f, 0.0222808f, 0.00000032f, -0.00016557f, 0.00009479f};
const static t_ff_gain ff_gain_search_turn_R_300 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0058330f, 0.0016753f, 0.0012683f, 0.0175503f, 0.000000321f, -0.00022961f, 0.00017918f};
const static t_ff_gain ff_gain_search_turn_L_300 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0064070f, 0.0015845f, 0.0013100f, 0.0144698f, 0.000000326f, -0.00006533f, 0.00005630f};
const static t_ff_gain ff_gain_search_turn_R_280 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0058360f, 0.0015999f, 0.0012282f, 0.0130955f, 0.000000187f, -0.00016090f, 0.00012841f};
const static t_ff_gain ff_gain_search_turn_L_280 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0068350f, 0.0015320f, 0.0012199f, 0.0068564f, 0.000000468f, -0.00004510f, 0.00003266f};

const static t_turn_param_table slalom_L90_table_400 = {0.40f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table_400 = {0.40f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search_400 = {&slalom_L90_table_400 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_400};
const static t_param param_R90_search_400 = {&slalom_R90_table_400, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_400};

const static t_turn_param_table slalom_L90_table_370 = {0.37f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table_370 = {0.37f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search_370 = {&slalom_L90_table_370 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_370};
const static t_param param_R90_search_370 = {&slalom_R90_table_370, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_370};

const static t_turn_param_table slalom_L90_table_350 = {0.35f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table_350 = {0.35f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search_350 = {&slalom_L90_table_350 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_350};
const static t_param param_R90_search_350 = {&slalom_R90_table_350, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_350};

const static t_turn_param_table slalom_L90_table_320 = {0.32f, 26.00f,9.46,11.16, 90.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_R90_table_320 = {0.32f,-26.00f,9.46,11.16,-90.0f,Turn_R,0.0f};
const static t_param param_L90_search_320 = {&slalom_L90_table_320 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_320};
const static t_param param_R90_search_320 = {&slalom_R90_table_320, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_320};

const static t_turn_param_table slalom_L90_table_300 = {0.30f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table_300 = {0.30f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search_300 = {&slalom_L90_table_300 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_300};
const static t_param param_R90_search_300 = {&slalom_R90_table_300, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_300};

const static t_turn_param_table slalom_L90_table_280 = {0.28f, 26.00f,9.49,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table_280 = {0.28f,-26.00f,9.49,11.16,-90.0f,Turn_R};
const static t_param param_L90_search_280 = {&slalom_L90_table_280 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_280};
const static t_param param_R90_search_280 = {&slalom_R90_table_280, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_280};


// Straight motion parameters for each velocity setting. The acceleration is
// determined by the velocity and the desired motion profile.

// Common non-suction SP feedforward for straight parameters at 700 mm/s and
// below. Search turns use the same SP coefficients.

const static t_ff_gain ff_gain_straight_non_suction = {
	FF_SP_VELO_COEF,
	FF_SP_ACCEL_COEF,
	FF_SP_BIAS_COEF,
	FF_OM_VELO_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_BIAS_COEF
};

const static t_ff_gain ff_gain_straight_suction = {
	FF_SP_VELO_COEF,
	FF_SP_ACCEL_COEF,
	FF_SP_BIAS_COEF,
	FF_OM_VELO_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_BIAS_COEF
};

const static t_pid_gain sp_gain_280 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_280 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_280 = {0.28f,4.0f};
const static t_straight_param st_param_280 = {&param_280,&sp_gain_280,&om_gain_280,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_300 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_300 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_320 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_320 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_320 = {0.32f,4.0f};
const static t_straight_param st_param_320 = {&param_320,&sp_gain_320,&om_gain_320,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_350 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_350 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_350 = {0.35f,4.0f};
const static t_straight_param st_param_350 = {&param_350,&sp_gain_350,&om_gain_350,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_370 = {2.00,0.02,0.00};
const static t_pid_gain om_gain_370 = {0.1f,0.005f,0.00f};
const static t_velo_param param_370 = {0.37f,4.0f};
const static t_straight_param st_param_370 = {&param_370,&sp_gain_370,&om_gain_370,&ff_gain_straight_non_suction};


const static t_pid_gain sp_gain_400 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_400 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_400 = {0.40f,4.0f};
const static t_straight_param st_param_400 = {&param_400,&sp_gain_400,&om_gain_400,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_450 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_450 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_500 = {2.00,0.02,0.00};//{10.0,0.05,0.00};//
const static t_pid_gain om_gain_500 = {0.1f,0.005f,0.00f};//{0.40f, 0.05f, 0.00f};//
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_600 = {2.00,0.02,0.00};
const static t_pid_gain om_gain_600 = {0.1f,0.005f,0.00f};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600,&ff_gain_straight_non_suction};

const static t_pid_gain sp_gain_700 = {2.00,0.02,0.00};
const static t_pid_gain om_gain_700 = {0.10f,0.01f,0.0f};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700,&ff_gain_straight_non_suction};


const static t_pid_gain sp_gain_1000 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1000 = {50.0f/1000.0f,1.80f/1000.0f,0.00f};
const static t_velo_param param_1000 = {1.0f,9.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

const static t_pid_gain sp_gain_1050 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1050 = {50.0f/1000.0f,1.80f/1000.0f,0.00f};
const static t_velo_param param_1050 = {1.05f,9.0f};
const static t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

const static t_pid_gain sp_gain_1100 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1100 = {0.15f, 0.005f, 0.00f};
const static t_velo_param param_1100 = {1.10f,9.0f};
const static t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};

const static t_pid_gain sp_gain_1200 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1200 = {0.07f, 0.005f, 0.00f};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

const static t_pid_gain sp_gain_1300 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1300 = {0.07f, 0.005f, 0.00f};
const static t_velo_param param_1300 = {1.30f,10.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

const static t_pid_gain sp_gain_1400 = {1.0,0.06,0.0};
const static t_pid_gain om_gain_1400 = {0.08f, 0.01f, 0.00f};
const static t_velo_param param_1400 = {1.40f,12.0f};
const static t_velo_param param_1400_acc2G = {1.40f,20.0f};
const static t_velo_param param_1400_acc3G = {1.40f,30.0f};
const static t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc2G  = {&param_1400_acc2G ,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc3G  = {&param_1400_acc3G ,&sp_gain_1400,&om_gain_1400};

const static t_pid_gain sp_gain_1500 = {1.0,0.05,0.0};
const static t_pid_gain om_gain_1500 = {0.2f, 0.01f, 0.00f};
const static t_velo_param param_1500 = {1.50f,12.0f};
const static t_velo_param param_1500_acc2G = {1.50f,20.0f};
const static t_velo_param param_1500_acc3G = {1.50f,30.0f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc2G  = {&param_1500_acc2G ,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc3G  = {&param_1500_acc3G ,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1600 = {2.0,0.05,0.00};
const static t_pid_gain om_gain_1600 = {0.2,0.01,0.0};
const static t_velo_param param_1600 = {1.60f,12.0f};
const static t_velo_param param_1600_acc2G = {1.60f,20.0f};
const static t_velo_param param_1600_acc3G = {1.60f,30.0f};
const static t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc2G  = {&param_1600_acc2G ,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc3G  = {&param_1600_acc3G ,&sp_gain_1600,&om_gain_1600};

const static t_pid_gain sp_gain_1700 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_1700 = {0.2,0.01,0.0};
const static t_velo_param param_1700 = {1.70f,12.0f};
const static t_velo_param param_1700_acc2G = {1.70f,20.0f};
const static t_velo_param param_1700_acc3G = {1.70f,30.0f};
const static t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc2G = {&param_1700_acc2G,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc3G = {&param_1700_acc3G,&sp_gain_1700,&om_gain_1700};

const static t_pid_gain sp_gain_1800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_1800 = {0.2,0.01,0.0};
const static t_velo_param param_1800 = {1.80f,12.0f};
const static t_velo_param param_1800_acc2G = {1.80f,20.0f};
const static t_velo_param param_1800_acc3G = {1.80f,30.0f};
const static t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc2G = {&param_1800_acc2G,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc3G = {&param_1800_acc3G,&sp_gain_1800,&om_gain_1800};


const static t_pid_gain sp_gain_1900 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_1900 = {0.2,0.01,0.0};
const static t_velo_param param_1900 = {1.90f,12.0f};
const static t_velo_param param_1900_acc2G = {1.90f,20.0f};
const static t_velo_param param_1900_acc3G = {1.90f,30.0f};
const static t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc2G = {&param_1900_acc2G,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc3G = {&param_1900_acc3G,&sp_gain_1900,&om_gain_1900};

const static t_pid_gain sp_gain_2000 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2000 = {0.2,0.01,0.0};
const static t_velo_param param_2000 = {2.0f,12.0f};
const static t_velo_param param_2000_acc2G = {2.0f,20.0f};
const static t_velo_param param_2000_acc3G = {2.0f,30.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc2G = {&param_2000_acc2G,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc3G = {&param_2000_acc3G,&sp_gain_2000,&om_gain_2000};


const static t_pid_gain sp_gain_2100 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2100 = {0.2,0.01,0.0};
const static t_velo_param param_2100 = {2.1f,12.0f};
const static t_velo_param param_2100_acc2G = {2.1f,20.0f};
const static t_velo_param param_2100_acc3G = {2.1f,30.0f};
const static t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc2G = {&param_2100_acc2G,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc3G = {&param_2100_acc3G,&sp_gain_2100,&om_gain_2100};

const static t_pid_gain sp_gain_2200 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2200 = {0.2,0.01,0.0};
const static t_velo_param param_2200 = {2.2f,12.0f};
const static t_velo_param param_2200_acc2G = {2.2f,20.0f};
const static t_velo_param param_2200_acc3G = {2.2f,30.0f};
const static t_straight_param st_param_2200 = {&param_2200,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc2G = {&param_2200_acc2G,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc3G = {&param_2200_acc3G,&sp_gain_2200,&om_gain_2200};

const static t_pid_gain sp_gain_2300 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2300 = {0.2,0.01,0.0};
const static t_velo_param param_2300 = {2.3f,12.0f};
const static t_velo_param param_2300_acc2G = {2.3f,20.0f};
const static t_velo_param param_2300_acc3G = {2.3f,30.0f};
const static t_straight_param st_param_2300 = {&param_2300,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc2G = {&param_2300_acc2G,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc3G = {&param_2300_acc3G,&sp_gain_2300,&om_gain_2300};

const static t_pid_gain sp_gain_2400 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2400 = {0.2,0.01,0.0};
const static t_velo_param param_2400 = {2.4f,12.0f};
const static t_velo_param param_2400_acc2G = {2.4f,20.0f};
const static t_velo_param param_2400_acc3G = {2.4f,30.0f};
const static t_straight_param st_param_2400 = {&param_2400,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc2G = {&param_2400_acc2G,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc3G = {&param_2400_acc3G,&sp_gain_2400,&om_gain_2400};

const static t_pid_gain sp_gain_2500 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2500 = {0.2,0.01,0.0};
const static t_velo_param param_2500 = {2.5f,12.0f};
const static t_velo_param param_2500_acc2G = {2.5f,20.0f};
const static t_velo_param param_2500_acc3G = {2.5f,30.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc2G = {&param_2500_acc2G,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc3G = {&param_2500_acc3G,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2600 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2600 = {0.2,0.01,0.0};
const static t_velo_param param_2600 = {2.6f,15.0f};
const static t_velo_param param_2600_acc2G = {2.6f,20.0f};
const static t_velo_param param_2600_acc3G = {2.6f,30.0f};
const static t_straight_param st_param_2600 = {&param_2600,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc2G = {&param_2600_acc2G,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc3G = {&param_2600_acc3G,&sp_gain_2600,&om_gain_2600};

const static t_pid_gain sp_gain_2700 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2700 = {0.2,0.01,0.0};
const static t_velo_param param_2700 = {2.7f,15.0f};
const static t_velo_param param_2700_acc2G = {2.7f,20.0f};
const static t_velo_param param_2700_acc3G = {2.7f,30.0f};
const static t_straight_param st_param_2700 = {&param_2700,&sp_gain_2700,&om_gain_2700};
const static t_straight_param st_param_2700_acc2G = {&param_2700_acc2G,&sp_gain_2700,&om_gain_2700};
const static t_straight_param st_param_2700_acc3G = {&param_2700_acc3G,&sp_gain_2700,&om_gain_2700};


const static t_pid_gain sp_gain_2800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2800 = {0.2,0.01,0.0};
const static t_velo_param param_2800 = {2.8f,15.0f};
const static t_velo_param param_2800_acc2G = {2.8f,20.0f};
const static t_velo_param param_2800_acc3G = {2.8f,30.0f};
const static t_straight_param st_param_2800 = {&param_2800,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc2G = {&param_2800_acc2G,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc3G = {&param_2800_acc3G,&sp_gain_2800,&om_gain_2800};


const static t_pid_gain sp_gain_2900 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_2900 = {0.2,0.01,0.0};
const static t_velo_param param_2900 = {2.9f,15.0f};
const static t_velo_param param_2900_acc2G = {2.9f,20.0f};
const static t_velo_param param_2900_acc3G = {2.9f,30.0f};
const static t_straight_param st_param_2900 = {&param_2900,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc2G = {&param_2900_acc2G,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc3G = {&param_2900_acc3G,&sp_gain_2900,&om_gain_2900};

const static t_pid_gain sp_gain_3000 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_3000 = {0.1,0.002,0.0};
const static t_velo_param param_3000 = {3.0f,15.0f};
const static t_velo_param param_3000_acc2G = {3.0f,20.0f};
const static t_velo_param param_3000_acc3G = {3.0f,30.0f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc2G = {&param_3000_acc2G,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc3G = {&param_3000_acc3G,&sp_gain_3000,&om_gain_3000};

const static t_pid_gain sp_gain_3200 = {2.0,0.05,0.01};
const static t_pid_gain om_gain_3200 = {0.1,0.002,0.0};
const static t_velo_param param_3200 = {3.2f,15.0f};
const static t_velo_param param_3200_acc2G = {3.2f,20.0f};
const static t_velo_param param_3200_acc3G = {3.2f,30.0f};
const static t_straight_param st_param_3200 = {&param_3200,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc2G = {&param_3200_acc2G,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc3G = {&param_3200_acc3G,&sp_gain_3200,&om_gain_3200};

const static t_pid_gain sp_gain_3400 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_3400 = {0.1,0.002,0.0};
const static t_velo_param param_3400 = {3.4f,15.0f};
const static t_velo_param param_3400_acc2G = {3.4f,20.0f};
const static t_velo_param param_3400_acc3G = {3.4f,30.0f};
const static t_straight_param st_param_3400 = {&param_3400,&sp_gain_3400,&om_gain_3400};
const static t_straight_param st_param_3400_acc2G = {&param_3400_acc2G,&sp_gain_3400,&om_gain_3400};
const static t_straight_param st_param_3400_acc3G = {&param_3400_acc3G,&sp_gain_3400,&om_gain_3400};

const static t_pid_gain sp_gain_3600 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_3600 = {0.1,0.002,0.0};
const static t_velo_param param_3600 = {3.6f,15.0f};
const static t_velo_param param_3600_acc2G = {3.6f,20.0f};
const static t_velo_param param_3600_acc3G = {3.6f,30.0f};
const static t_straight_param st_param_3600 = {&param_3600,&sp_gain_3600,&om_gain_3600};
const static t_straight_param st_param_3600_acc2G = {&param_3600_acc2G,&sp_gain_3600,&om_gain_3600};
const static t_straight_param st_param_3600_acc3G = {&param_3600_acc3G,&sp_gain_3600,&om_gain_3600};

const static t_pid_gain sp_gain_3800 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_3800 = {0.1,0.002,0.0};
const static t_velo_param param_3800 = {3.8f,15.0f};
const static t_velo_param param_3800_acc2G = {3.8f,20.0f};
const static t_velo_param param_3800_acc3G = {3.8f,30.0f};
const static t_straight_param st_param_3800 = {&param_3800,&sp_gain_3800,&om_gain_3800};
const static t_straight_param st_param_3800_acc2G = {&param_3800_acc2G,&sp_gain_3800,&om_gain_3800};
const static t_straight_param st_param_3800_acc3G = {&param_3800_acc3G,&sp_gain_3800,&om_gain_3800};

const static t_pid_gain sp_gain_4000 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_4000 = {0.1,0.002,0.0};
const static t_velo_param param_4000 = {4.0f,15.0f};
const static t_velo_param param_4000_acc2G = {4.0f,20.0f};
const static t_velo_param param_4000_acc3G = {4.0f,30.0f};
const static t_straight_param st_param_4000 = {&param_4000,&sp_gain_4000,&om_gain_4000};
const static t_straight_param st_param_4000_acc2G = {&param_4000_acc2G,&sp_gain_4000,&om_gain_4000};
const static t_straight_param st_param_4000_acc3G = {&param_4000_acc3G,&sp_gain_4000,&om_gain_4000};

const static t_pid_gain sp_gain_4200 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_4200 = {0.1,0.002,0.0};
const static t_velo_param param_4200 = {4.2f,15.0f};
const static t_velo_param param_4200_acc2G = {4.2f,20.0f};
const static t_velo_param param_4200_acc3G = {4.2f,30.0f};
const static t_straight_param st_param_4200 = {&param_4200,&sp_gain_4200,&om_gain_4200};
const static t_straight_param st_param_4200_acc2G = {&param_4200_acc2G,&sp_gain_4200,&om_gain_4200};
const static t_straight_param st_param_4200_acc3G = {&param_4200_acc3G,&sp_gain_4200,&om_gain_4200};

const static t_pid_gain sp_gain_4400 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_4400 = {0.1,0.002,0.0};
const static t_velo_param param_4400 = {4.4f,15.0f};
const static t_velo_param param_4400_acc2G = {4.4f,20.0f};
const static t_velo_param param_4400_acc3G = {4.4f,30.0f};
//const static t_velo_param param_4600_acc5G = {4.4f,50.0f};
const static t_straight_param st_param_4400 = {&param_4400,&sp_gain_4400,&om_gain_4400};
const static t_straight_param st_param_4400_acc2G = {&param_4400_acc2G,&sp_gain_4400,&om_gain_4400};
const static t_straight_param st_param_4400_acc3G = {&param_4400_acc3G,&sp_gain_4400,&om_gain_4400};

const static t_pid_gain sp_gain_4600 = {2.0,0.05,0.0};
const static t_pid_gain om_gain_4600 = {0.1,0.002,0.0};
const static t_velo_param param_4600 = {4.6f,15.0f};
const static t_velo_param param_4600_acc2G = {4.6f,20.0f};
const static t_velo_param param_4600_acc3G = {4.6f,30.0f};
//const static t_velo_param param_4600_acc5G = {4.6f,50.0f};
const static t_straight_param st_param_4600 = {&param_4600,&sp_gain_4600,&om_gain_4600};
const static t_straight_param st_param_4600_acc2G = {&param_4600_acc2G,&sp_gain_4600,&om_gain_4600};
const static t_straight_param st_param_4600_acc3G = {&param_4600_acc3G,&sp_gain_4600,&om_gain_4600};

const static t_velo_param param_1400_acc5G = {1.40f, 50.0f};
const static t_straight_param st_param_1400_acc5G = {&param_1400_acc5G, &sp_gain_1400, &om_gain_1400};

const static t_velo_param param_1500_acc5G = {1.50f, 50.0f};
const static t_straight_param st_param_1500_acc5G = {&param_1500_acc5G, &sp_gain_1500, &om_gain_1500};

const static t_velo_param param_1600_acc5G = {1.60f, 50.0f};
const static t_straight_param st_param_1600_acc5G = {&param_1600_acc5G, &sp_gain_1600, &om_gain_1600};

const static t_velo_param param_1700_acc5G = {1.70f, 50.0f};
const static t_straight_param st_param_1700_acc5G = {&param_1700_acc5G, &sp_gain_1700, &om_gain_1700};

const static t_velo_param param_1800_acc5G = {1.80f, 50.0f};
const static t_straight_param st_param_1800_acc5G = {&param_1800_acc5G, &sp_gain_1800, &om_gain_1800};

const static t_velo_param param_1900_acc5G = {1.90f, 50.0f};
const static t_straight_param st_param_1900_acc5G = {&param_1900_acc5G, &sp_gain_1900, &om_gain_1900};

const static t_velo_param param_2000_acc5G = {2.00f, 50.0f};
const static t_straight_param st_param_2000_acc5G = {&param_2000_acc5G, &sp_gain_2000, &om_gain_2000};

const static t_velo_param param_2100_acc5G = {2.10f, 50.0f};
const static t_straight_param st_param_2100_acc5G = {&param_2100_acc5G, &sp_gain_2100, &om_gain_2100};

const static t_velo_param param_2200_acc5G = {2.20f, 50.0f};
const static t_straight_param st_param_2200_acc5G = {&param_2200_acc5G, &sp_gain_2200, &om_gain_2200};

const static t_velo_param param_2300_acc5G = {2.30f, 50.0f};
const static t_straight_param st_param_2300_acc5G = {&param_2300_acc5G, &sp_gain_2300, &om_gain_2300};

const static t_velo_param param_2400_acc5G = {2.40f, 50.0f};
const static t_straight_param st_param_2400_acc5G = {&param_2400_acc5G, &sp_gain_2400, &om_gain_2400};

const static t_velo_param param_2500_acc5G = {2.50f, 50.0f};
const static t_straight_param st_param_2500_acc5G = {&param_2500_acc5G, &sp_gain_2500, &om_gain_2500};

const static t_velo_param param_2600_acc5G = {2.60f, 50.0f};
const static t_straight_param st_param_2600_acc5G = {&param_2600_acc5G, &sp_gain_2600, &om_gain_2600};

const static t_velo_param param_2700_acc5G = {2.70f, 50.0f};
const static t_straight_param st_param_2700_acc5G = {&param_2700_acc5G, &sp_gain_2700, &om_gain_2700};

const static t_velo_param param_2800_acc5G = {2.80f, 50.0f};
const static t_straight_param st_param_2800_acc5G = {&param_2800_acc5G, &sp_gain_2800, &om_gain_2800};

const static t_velo_param param_2900_acc5G = {2.90f, 50.0f};
const static t_straight_param st_param_2900_acc5G = {&param_2900_acc5G, &sp_gain_2900, &om_gain_2900};

const static t_velo_param param_3000_acc5G = {3.00f, 50.0f};
const static t_straight_param st_param_3000_acc5G = {&param_3000_acc5G, &sp_gain_3000, &om_gain_3000};

const static t_velo_param param_3200_acc5G = {3.20f, 50.0f};
const static t_straight_param st_param_3200_acc5G = {&param_3200_acc5G, &sp_gain_3200, &om_gain_3200};

const static t_velo_param param_3400_acc5G = {3.40f, 50.0f};
const static t_straight_param st_param_3400_acc5G = {&param_3400_acc5G, &sp_gain_3400, &om_gain_3400};

const static t_velo_param param_3600_acc5G = {3.60f, 50.0f};
const static t_straight_param st_param_3600_acc5G = {&param_3600_acc5G, &sp_gain_3600, &om_gain_3600};

const static t_velo_param param_3800_acc5G = {3.80f, 50.0f};
const static t_straight_param st_param_3800_acc5G = {&param_3800_acc5G, &sp_gain_3800, &om_gain_3800};

const static t_velo_param param_4000_acc5G = {4.00f, 50.0f};
const static t_straight_param st_param_4000_acc5G = {&param_4000_acc5G, &sp_gain_4000, &om_gain_4000};

const static t_velo_param param_4200_acc5G = {4.20f, 50.0f};
const static t_straight_param st_param_4200_acc5G = {&param_4200_acc5G, &sp_gain_4200, &om_gain_4200};

const static t_velo_param param_4400_acc5G = {4.40f, 50.0f};
const static t_straight_param st_param_4400_acc5G = {&param_4400_acc5G, &sp_gain_4400, &om_gain_4400};

const static t_velo_param param_4600_acc5G = {4.60f, 50.0f};
const static t_straight_param st_param_4600_acc5G = {&param_4600_acc5G, &sp_gain_4600, &om_gain_4600};





const static t_straight_param *const st_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const st_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const st_mode_700_v0[] = {&st_param_700};
const static t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const st_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
														  &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000	};

const static t_straight_param *const st_mode_1400_v0[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};

const static t_straight_param *const st_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000,&st_param_3200	,&st_param_3400		};
const static t_straight_param *const st_mode_1400_v2[] = {	&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};
const static t_straight_param *const st_mode_1400_v3[] = {	&st_param_1400_acc3G,&st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};



const static t_straight_param *const st_mode_1500_v0[] = {	&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000		};
const static t_straight_param *const st_mode_1500_v1[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G		};

const static t_straight_param *const st_mode_1500_v2[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};


const static t_straight_param *const st_mode_1600_v1[] = {	&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const st_mode_1600_v2[] = {  &st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};

const static t_straight_param *const st_mode_1600_v3[] = {
	    &st_param_1600_acc5G, &st_param_1800_acc5G, &st_param_2000_acc5G, &st_param_2200_acc5G, &st_param_2400_acc5G,
	    &st_param_2600_acc5G, &st_param_2800_acc5G, &st_param_3000_acc5G, &st_param_3200_acc5G, &st_param_3400_acc5G,
	    &st_param_3600_acc5G, &st_param_3800_acc5G, &st_param_4000_acc5G, &st_param_4200_acc5G, &st_param_4400_acc5G,
	    &st_param_4600_acc5G,
	};
const static t_straight_param *const st_mode_1800_v1[] = {  &st_param_1800_acc3G,&st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};

const static t_straight_param *const st_mode_2000_v1[] = {  &st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};

const static t_straight_param *const di_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const di_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const di_mode_700_v0[] = {&st_param_700};
const static t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const di_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
		  	  	  	  	  	  	  	  	  	  	  	  	  &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1400_v2[] =  {	&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_1500_v0[] = {&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
														  &st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1500_v1[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G		};
const static t_straight_param *const di_mode_1500_v2[] =  {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_1600_v1[] =  {	&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_1600_v2[] = {  &st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};


const static t_straight_param *const di_mode_1800_v1[] =  { &st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_2000_v1[] = {  &st_param_2000_acc3G,&st_param_2200_acc3G,&st_param_2400_acc3G,
															&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,&st_param_3400_acc3G,&st_param_3600_acc3G,
															&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,&st_param_4600_acc3G,};



const static t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const static t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const static t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,Turn_L};
const static t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};


const static t_param *const *const acc_mode_1400[] = {mode_1400_acc,mode_1600_acc_v1};
const static t_param *const *const acc_mode_1600_v1[] = {mode_1600_acc_v2,mode_1800_acc_v1};
const static t_param *const *const acc_mode_1600_v2[] = {mode_1600_acc_v2,mode_1800_acc_v1};
const static t_param *const *const acc_mode_1600_v3[] = {mode_1600_acc_v2,mode_1800_acc_v2,mode_2000_acc};
const static t_param *const *const acc_mode_1800_v1[] = {mode_1800,mode_2000_acc_v2};



#endif /* RUN_PARAM_A_H_ */
