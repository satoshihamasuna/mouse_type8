// 機体Bの直進・共通パラメータ定義。extern宣言に対して一度だけリンクする。
// 速度条件とPID・FFは新形式で定義し、旧速度別定義はコメントで保持する。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// 2026-11-18: 新形式のPID・FF。速度候補/加減速度から独立させ、同じ組を共有する。
// 2026-11-18 機体B: 2200 mm/s以上は速度PID・角速度PID・FFを1組に統一。
// 速度・加減速度は先頭のt_speed_profileから生成し、以下の旧速度別定義は使用しない。
constexpr t_control_gain straight_control_high = {
	{4.0f,0.05f,0.0f},
	{0.2f,0.01f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_low = {
	{1.2f,0.02f,0.0f},
	{0.1f,0.005f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_500 = {
	{1.2f,0.02f,0.0f},
	{0.1f,0.01f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_700 = {
	{1.5f,0.05f,0.0f},
	{0.1f,0.005f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_1000 = {
	{3.0f,0.05f,0.0f},
	{0.1f,0.005f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_1200 = {
	{3.0f,0.05f,0.0f},
	{0.2f,0.01f,0.0f},
	&ff_gain_default
};

constexpr t_control_gain straight_gain_1400 = {
	{3.0f,0.05f,0.0f},
	{0.2f,0.005f,0.0f},
	&ff_gain_default
};

constexpr t_control_band control_bands_straight_300_v0[] = {
	{0.3f,&straight_gain_low},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_300_v0 = make_control_profile(control_bands_straight_300_v0);
constexpr t_control_band control_bands_straight_300_v1[] = {
	{0.3f,&straight_gain_low},
	{0.5f,&straight_gain_500},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_300_v1 = make_control_profile(control_bands_straight_300_v1);
constexpr t_control_band control_bands_straight_plain500[] = {
	{0.5f,&straight_gain_500},
	{0.6f,&straight_gain_low},
	{0.7f,&straight_gain_700},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_plain500 = make_control_profile(control_bands_straight_plain500);
constexpr t_control_band control_bands_straight_700_v0[] = {
	{0.7f,&straight_gain_700},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_700_v0 = make_control_profile(control_bands_straight_700_v0);
constexpr t_control_band control_bands_straight_1000_v0[] = {
	{1.0f,&straight_gain_1000},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_1000_v0 = make_control_profile(control_bands_straight_1000_v0);
constexpr t_control_band control_bands_straight_1000_v1[] = {
	{1.0f,&straight_gain_1000},
	{1.2f,&straight_gain_1200},
	{1.4f,&straight_gain_1400},
	{2.0f,&straight_control_high},
};

const t_control_profile straight_control_1000_v1 = make_control_profile(control_bands_straight_1000_v1);
constexpr t_control_band control_bands_straight_1200_v0[] = {
	{1.2f,&straight_gain_1200},
	{1.4f,&straight_gain_1400},
	{2.2f,&straight_control_high},
};

const t_control_profile straight_control_1200_v0 = make_control_profile(control_bands_straight_1200_v0);
constexpr t_control_band control_bands_straight_1200_v1[] = {
	{1.2f,&straight_gain_1200},
	{1.4f,&straight_gain_1400},
	{1.6f,&straight_control_high},
};

const t_control_profile straight_control_1200_v1 = make_control_profile(control_bands_straight_1200_v1);
constexpr t_control_band control_bands_straight_1400_v0[] = {
	{1.4f,&straight_gain_1400},
	{1.6f,&straight_control_high},
};

const t_control_profile straight_control_1400_v0 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile straight_control_1400_v1 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile straight_control_1400_v2 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile straight_control_1400_v3 = make_control_profile(control_bands_straight_1400_v0);
constexpr t_control_band control_bands_straight_1500_v0[] = {
	{1.5f,&straight_gain_1400},
	{1.6f,&straight_control_high},
};

const t_control_profile straight_control_1500_v0 = make_control_profile(control_bands_straight_1500_v0);
const t_control_profile straight_control_1500_v1 = make_control_profile(control_bands_straight_1500_v0);
const t_control_profile straight_control_1500_v2 = make_control_profile(control_bands_straight_1500_v0);
constexpr t_control_band control_bands_straight_1600_v1[] = {
	{1.6f,&straight_control_high},
};

const t_control_profile straight_control_1600_v1 = make_control_profile(control_bands_straight_1600_v1);
const t_control_profile straight_control_1600_v2 = make_control_profile(control_bands_straight_1600_v1);
const t_control_profile straight_control_1600_v3 = make_control_profile(control_bands_straight_1600_v1);
constexpr t_control_band control_bands_straight_1800_v1[] = {
	{1.8f,&straight_control_high},
};

const t_control_profile straight_control_1800_v1 = make_control_profile(control_bands_straight_1800_v1);
constexpr t_control_band control_bands_straight_2000_v1[] = {
	{2.0f,&straight_control_high},
};

const t_control_profile straight_control_2000_v1 = make_control_profile(control_bands_straight_2000_v1);
const t_control_profile diagonal_control_300_v0 = make_control_profile(control_bands_straight_300_v0);
const t_control_profile diagonal_control_300_v1 = make_control_profile(control_bands_straight_300_v1);
const t_control_profile diagonal_control_plain500 = make_control_profile(control_bands_straight_plain500);
const t_control_profile diagonal_control_700_v0 = make_control_profile(control_bands_straight_700_v0);
const t_control_profile diagonal_control_1000_v0 = make_control_profile(control_bands_straight_1000_v0);
const t_control_profile diagonal_control_1000_v1 = make_control_profile(control_bands_straight_1000_v1);
const t_control_profile diagonal_control_1200_v0 = make_control_profile(control_bands_straight_1200_v0);
const t_control_profile diagonal_control_1200_v1 = make_control_profile(control_bands_straight_1200_v1);
const t_control_profile diagonal_control_1400_v0 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile diagonal_control_1400_v1 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile diagonal_control_1400_v2 = make_control_profile(control_bands_straight_1400_v0);
const t_control_profile diagonal_control_1500_v0 = make_control_profile(control_bands_straight_1500_v0);
const t_control_profile diagonal_control_1500_v1 = make_control_profile(control_bands_straight_1500_v0);
const t_control_profile diagonal_control_1500_v2 = make_control_profile(control_bands_straight_1500_v0);
const t_control_profile diagonal_control_1600_v1 = make_control_profile(control_bands_straight_1600_v1);
const t_control_profile diagonal_control_1600_v2 = make_control_profile(control_bands_straight_1600_v1);
const t_control_profile diagonal_control_1800_v1 = make_control_profile(control_bands_straight_1800_v1);
const t_control_profile diagonal_control_2000_v1 = make_control_profile(control_bands_straight_2000_v1);

// 探索・デバッグの固定速度条件。単位は速度m/s、加速度m/s^2。
// 2026-11-21 ユーザー指定により通常探索の直進・ターンを320 mm/sに設定。
constexpr t_motion_profile search_straight = {{0.32f,4.0f},&straight_gain_low};
constexpr t_motion_profile search_straight_slow = {{0.32f,4.0f},&straight_gain_low};
constexpr t_motion_profile debug_straight_500 = {{0.5f,6.0f},&straight_gain_500};
constexpr t_motion_profile debug_straight_700 = {{0.7f,6.0f},&straight_gain_700};
constexpr t_motion_profile debug_straight_1600 = {{1.6f,20.0f},&straight_control_high};
constexpr t_motion_profile debug_straight_1800 = {{1.8f,30.0f},&straight_control_high};
constexpr t_motion_profile debug_straight_2000 = {{2.0f,30.0f},&straight_control_high};


// 2026-11-17 機体B: plain500の既存条件を新形式へ移行。
// 順序: base[m/s], accel[m/s^2], decel[m/s^2], step[m/s], max[m/s]。
// PID/FFはstraight_control_plain500 / diagonal_control_plain500を使う。
const t_speed_profile straight_speed_plain500 = {0.50f, 6.0f, 6.0f, 0.10f, 0.70f};
const t_speed_profile diagonal_speed_plain500 = {0.50f, 6.0f, 6.0f, 0.10f, 0.70f};

// 2026-11-17 機体B: 全38設定を機体Bの既存候補・加減速度から整理。
// 単一範囲は5項目で編集。不等間隔/加速度切替はconstexprの範囲表を編集する。
// 各候補の加減速度を区間全体へ適用する（速度に応じた連続切替ではない）。
const t_speed_profile straight_speed_300_v0 = {0.30f, 4.0f, 4.0f, 0.00f, 0.30f};

constexpr t_speed_range straight_speed_ranges_300_v1[] = {
	{0.30f, 4.0f, 4.0f, 0.00f, 0.30f},
	{0.50f, 6.0f, 6.0f, 0.00f, 0.50f},
};
const t_speed_profile straight_speed_300_v1 = make_speed_profile(straight_speed_ranges_300_v1);

const t_speed_profile straight_speed_700_v0 = {0.70f, 6.0f, 6.0f, 0.00f, 0.70f};

const t_speed_profile straight_speed_1000_v0 = {1.00f, 9.0f, 9.0f, 0.00f, 1.00f};

constexpr t_speed_range straight_speed_ranges_1000_v1[] = {
	{1.00f, 9.0f, 9.0f, 0.10f, 1.10f},
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.50f},
	{2.00f, 12.0f, 12.0f, 0.00f, 2.00f},
};
const t_speed_profile straight_speed_1000_v1 = make_speed_profile(straight_speed_ranges_1000_v1);

constexpr t_speed_range straight_speed_ranges_1200_v0[] = {
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.50f},
};
const t_speed_profile straight_speed_1200_v0 = make_speed_profile(straight_speed_ranges_1200_v0);

constexpr t_speed_range straight_speed_ranges_1200_v1[] = {
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.60f},
	{1.80f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile straight_speed_1200_v1 = make_speed_profile(straight_speed_ranges_1200_v1);

constexpr t_speed_range straight_speed_ranges_1400_v0[] = {
	{1.40f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile straight_speed_1400_v0 = make_speed_profile(straight_speed_ranges_1400_v0);

constexpr t_speed_range straight_speed_ranges_1400_v1[] = {
	{1.40f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.40f},
};
const t_speed_profile straight_speed_1400_v1 = make_speed_profile(straight_speed_ranges_1400_v1);

const t_speed_profile straight_speed_1400_v2 = {1.40f, 20.0f, 20.0f, 0.20f, 4.60f};

const t_speed_profile straight_speed_1400_v3 = {1.40f, 30.0f, 30.0f, 0.20f, 4.60f};

constexpr t_speed_range straight_speed_ranges_1500_v0[] = {
	{1.50f, 12.0f, 12.0f, 0.10f, 1.60f},
	{1.80f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile straight_speed_1500_v0 = make_speed_profile(straight_speed_ranges_1500_v0);

constexpr t_speed_range straight_speed_ranges_1500_v1[] = {
	{1.50f, 20.0f, 20.0f, 0.10f, 1.60f},
	{1.80f, 20.0f, 20.0f, 0.20f, 3.00f},
};
const t_speed_profile straight_speed_1500_v1 = make_speed_profile(straight_speed_ranges_1500_v1);

constexpr t_speed_range straight_speed_ranges_1500_v2[] = {
	{1.50f, 20.0f, 20.0f, 0.10f, 1.60f},
	{1.80f, 20.0f, 20.0f, 0.20f, 4.60f},
};
const t_speed_profile straight_speed_1500_v2 = make_speed_profile(straight_speed_ranges_1500_v2);

const t_speed_profile straight_speed_1600_v1 = {1.60f, 20.0f, 20.0f, 0.20f, 4.60f};

const t_speed_profile straight_speed_1600_v2 = {1.60f, 30.0f, 30.0f, 0.20f, 4.60f};

const t_speed_profile straight_speed_1600_v3 = {1.60f, 50.0f, 50.0f, 0.20f, 4.60f};

const t_speed_profile straight_speed_1800_v1 = {1.80f, 30.0f, 30.0f, 0.20f, 4.60f};

const t_speed_profile straight_speed_2000_v1 = {2.00f, 30.0f, 30.0f, 0.20f, 4.60f};

const t_speed_profile diagonal_speed_300_v0 = {0.30f, 4.0f, 4.0f, 0.00f, 0.30f};

constexpr t_speed_range diagonal_speed_ranges_300_v1[] = {
	{0.30f, 4.0f, 4.0f, 0.00f, 0.30f},
	{0.50f, 6.0f, 6.0f, 0.00f, 0.50f},
};
const t_speed_profile diagonal_speed_300_v1 = make_speed_profile(diagonal_speed_ranges_300_v1);

const t_speed_profile diagonal_speed_700_v0 = {0.70f, 6.0f, 6.0f, 0.00f, 0.70f};

const t_speed_profile diagonal_speed_1000_v0 = {1.00f, 9.0f, 9.0f, 0.00f, 1.00f};

constexpr t_speed_range diagonal_speed_ranges_1000_v1[] = {
	{1.00f, 9.0f, 9.0f, 0.10f, 1.10f},
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.50f},
	{2.00f, 12.0f, 12.0f, 0.00f, 2.00f},
};
const t_speed_profile diagonal_speed_1000_v1 = make_speed_profile(diagonal_speed_ranges_1000_v1);

constexpr t_speed_range diagonal_speed_ranges_1200_v0[] = {
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.50f},
};
const t_speed_profile diagonal_speed_1200_v0 = make_speed_profile(diagonal_speed_ranges_1200_v0);

constexpr t_speed_range diagonal_speed_ranges_1200_v1[] = {
	{1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
	{1.40f, 12.0f, 12.0f, 0.10f, 1.60f},
	{1.80f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile diagonal_speed_1200_v1 = make_speed_profile(diagonal_speed_ranges_1200_v1);

const t_speed_profile diagonal_speed_1400_v0 = {1.40f, 12.0f, 12.0f, 0.20f, 2.00f};

constexpr t_speed_range diagonal_speed_ranges_1400_v1[] = {
	{1.40f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile diagonal_speed_1400_v1 = make_speed_profile(diagonal_speed_ranges_1400_v1);

const t_speed_profile diagonal_speed_1400_v2 = {1.40f, 20.0f, 20.0f, 0.20f, 4.60f};

constexpr t_speed_range diagonal_speed_ranges_1500_v0[] = {
	{1.50f, 12.0f, 12.0f, 0.10f, 1.60f},
	{1.80f, 12.0f, 12.0f, 0.20f, 2.40f},
	{2.60f, 15.0f, 15.0f, 0.20f, 3.00f},
};
const t_speed_profile diagonal_speed_1500_v0 = make_speed_profile(diagonal_speed_ranges_1500_v0);

constexpr t_speed_range diagonal_speed_ranges_1500_v1[] = {
	{1.50f, 20.0f, 20.0f, 0.10f, 1.60f},
	{1.80f, 20.0f, 20.0f, 0.20f, 3.00f},
};
const t_speed_profile diagonal_speed_1500_v1 = make_speed_profile(diagonal_speed_ranges_1500_v1);

constexpr t_speed_range diagonal_speed_ranges_1500_v2[] = {
	{1.50f, 20.0f, 20.0f, 0.10f, 1.60f},
	{1.80f, 20.0f, 20.0f, 0.20f, 4.60f},
};
const t_speed_profile diagonal_speed_1500_v2 = make_speed_profile(diagonal_speed_ranges_1500_v2);

const t_speed_profile diagonal_speed_1600_v1 = {1.60f, 20.0f, 20.0f, 0.20f, 4.60f};

const t_speed_profile diagonal_speed_1600_v2 = {1.60f, 30.0f, 30.0f, 0.20f, 4.60f};

const t_speed_profile diagonal_speed_1800_v1 = {1.80f, 20.0f, 20.0f, 0.20f, 4.60f};

const t_speed_profile diagonal_speed_2000_v1 = {2.00f, 30.0f, 30.0f, 0.20f, 4.60f};


// Params/typedef_run_param.h の既定FF。機体選択済みの係数を使う。
const t_ff_gain ff_gain_default = {
	FF_SP_VELO_COEF,
	FF_SP_ACCEL_COEF,
	FF_SP_BIAS_COEF,
	FF_OM_VELO_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_BIAS_COEF
};

// ===== Params/Param_B/run_param_B.h =====
/*
 * run_param_B.h
 *
 *  Created on: Nov 2, 2025
 *      Author: sato1
 */


#include "Params/Param_B/turn_1000/turn_1000.h"
#include "Params/Param_B/turn_1200/turn_1200.h"
#include "Params/Param_B/turn_1400/turn_1400.h"
#include "Params/Param_B/turn_1500/turn_1500.h"
#include "Params/Param_B/turn_1600/turn_1600.h"
#include "Params/Param_B/turn_1800/turn_1800.h"
#include "Params/Param_B/turn_2000/turn_2000.h"
#include "Params/Param_B/turn_300/turn_300.h"
#include "Params/Param_B/turn_500/turn_500.h"
#include "Params/Param_B/turn_700/turn_700.h"
#include "Component/Inc/controller.h"
#include "Module/Inc/vehicle.h"
#include "Params/typedef_run_param.h"

const t_pid_gain basic_sp_gain = {1.20,0.02,0.0};
const t_pid_gain basic_om_gain = {0.1f,0.005f,0.00f};//{0.05f, 0.001f, 0.00f};

// Bの吸引開始は未計測。B自身の基本ゲインを初期値として保持する。
const t_pid_gain om_gain_suction_start = {0.1f,0.005f,0.0f};

const t_pid_gain search_sp_gain = {1.200,0.02,0.0};
const t_pid_gain search_om_gain = {0.1f,0.005f,0.00f};

// MOUSE_B has no separately identified suction plant yet.  Keep the named
// profile available so suction straight-state selection is shared by both
// hardware variants without changing B's existing feedforward behavior.
const t_ff_gain ff_gain_straight_suction = {
	FF_SP_VELO_COEF,
	FF_SP_ACCEL_COEF,
	FF_SP_BIAS_COEF,
	FF_OM_VELO_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_ACCEL_COEF,
	FF_OM_BIAS_COEF
};

// Feedback gains used exclusively by pivot turns.
const t_pid_gain sp_gain_pivot_turn = {1.20f, 0.02f, 0.0f};
const t_pid_gain om_gain_pivot_turn = {0.1f, 0.005f, 0.0f};

// MOUSE_B pivot feedforward has not been identified from logs yet.
const t_ff_gain ff_gain_pivot_turn_R = {
	FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_OM_ACCEL_COEF, FF_OM_BIAS_COEF
};
const t_ff_gain ff_gain_pivot_turn_L = {
	FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_OM_ACCEL_COEF, FF_OM_BIAS_COEF
};

#include "Params/Param_B/turn_search/turn_search.h"

// const t_pid_gain sp_gain_280 = {1.20,0.02,0.0};
// const t_pid_gain om_gain_280 = {0.1f,0.005f,0.00f};//{
// const t_velo_param param_280 = {0.28f,4.0f};
// const t_straight_param st_param_280 = {&param_280,&sp_gain_280,&om_gain_280};

// const t_pid_gain sp_gain_300 = {1.20,0.02,0.0};//{1.0,0.05,0.02};
// const t_pid_gain om_gain_300 = {0.1f,0.005f,0.00f};//{
// const t_velo_param param_300 = {0.30f,4.0f};
// const t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

// const t_pid_gain sp_gain_320 = {1.20,0.02,0.0};//{1.0,0.05,0.02};
// const t_pid_gain om_gain_320 = {0.1f,0.005f,0.00f};//{
// const t_velo_param param_320 = {0.32f,4.0f};
// const t_straight_param st_param_320 = {&param_320,&sp_gain_320,&om_gain_320};

// const t_pid_gain sp_gain_450 = {1.20,0.02,0.0};//{1.0,0.05,0.02};
// const t_pid_gain om_gain_450 = {0.1f,0.005f,0.00f};//{
// const t_velo_param param_450 = {0.45f,6.0f};
// const t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

// const t_pid_gain sp_gain_500 = {1.20,0.02,0.0};
// const t_pid_gain om_gain_500 = {0.1f,0.01f,0.00f};//{
// const t_velo_param param_500 = {0.50f,6.0f};
// const t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500};

// const t_pid_gain sp_gain_600 = {1.20,0.02,0.0};
// const t_pid_gain om_gain_600 = {0.1f,0.005f,0.00f};
// const t_velo_param param_600 = {0.60f,6.0f};
// const t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

// const t_pid_gain sp_gain_700 = {1.5,0.05,0.00};
// const t_pid_gain om_gain_700 = {0.1f,0.005f,0.00f};
// const t_velo_param param_700 = {0.70f,6.0f};
// const t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};


// const t_pid_gain sp_gain_1000 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1000 = {0.1f,0.005f,0.00f};
// const t_velo_param param_1000 = {1.0f,9.0f};
// const t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

// const t_pid_gain sp_gain_1050 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1050 = {0.1f,0.005f,0.00f};
// const t_velo_param param_1050 = {1.05f,9.0f};
// const t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

// const t_pid_gain sp_gain_1100 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1100 = {0.1f,0.005f,0.00f};
// const t_velo_param param_1100 = {1.10f,9.0f};
// const t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};

// const t_pid_gain sp_gain_1200 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1200 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1200 = {1.20f,10.0f};
// const t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

// const t_pid_gain sp_gain_1300 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1300 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1300 = {1.30f,10.0f};
// const t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

// const t_pid_gain sp_gain_1400 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1400 = {0.2f, 0.005f, 0.00f};
// const t_velo_param param_1400 = {1.40f,12.0f};
// const t_velo_param param_1400_acc2G = {1.40f,20.0f};
// const t_velo_param param_1400_acc3G = {1.40f,30.0f};
// const t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};
// const t_straight_param st_param_1400_acc2G  = {&param_1400_acc2G ,&sp_gain_1400,&om_gain_1400};
// const t_straight_param st_param_1400_acc3G  = {&param_1400_acc3G ,&sp_gain_1400,&om_gain_1400};

// const t_pid_gain sp_gain_1500 = {3.0,0.05,0.0};
// const t_pid_gain om_gain_1500 = {0.2f, 0.005f, 0.00f};
// const t_velo_param param_1500 = {1.50f,12.0f};
// const t_velo_param param_1500_acc2G = {1.50f,20.0f};
// const t_velo_param param_1500_acc3G = {1.50f,30.0f};
// const t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};
// const t_straight_param st_param_1500_acc2G  = {&param_1500_acc2G ,&sp_gain_1500,&om_gain_1500};
// const t_straight_param st_param_1500_acc3G  = {&param_1500_acc3G ,&sp_gain_1500,&om_gain_1500};

// const t_pid_gain sp_gain_1600 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_1600 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1600 = {1.60f,12.0f};
// const t_velo_param param_1600_acc2G = {1.60f,20.0f};
// const t_velo_param param_1600_acc3G = {1.60f,30.0f};
// const t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};
// const t_straight_param st_param_1600_acc2G  = {&param_1600_acc2G ,&sp_gain_1600,&om_gain_1600};
// const t_straight_param st_param_1600_acc3G  = {&param_1600_acc3G ,&sp_gain_1600,&om_gain_1600};

// const t_pid_gain sp_gain_1700 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_1700 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1700 = {1.70f,12.0f};
// const t_velo_param param_1700_acc2G = {1.70f,20.0f};
// const t_velo_param param_1700_acc3G = {1.70f,30.0f};
// const t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};
// const t_straight_param st_param_1700_acc2G = {&param_1700_acc2G,&sp_gain_1700,&om_gain_1700};
// const t_straight_param st_param_1700_acc3G = {&param_1700_acc3G,&sp_gain_1700,&om_gain_1700};

// const t_pid_gain sp_gain_1800 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_1800 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1800 = {1.80f,12.0f};
// const t_velo_param param_1800_acc2G = {1.80f,20.0f};
// const t_velo_param param_1800_acc3G = {1.80f,30.0f};
// const t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};
// const t_straight_param st_param_1800_acc2G = {&param_1800_acc2G,&sp_gain_1800,&om_gain_1800};
// const t_straight_param st_param_1800_acc3G = {&param_1800_acc3G,&sp_gain_1800,&om_gain_1800};


// const t_pid_gain sp_gain_1900 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_1900 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_1900 = {1.90f,12.0f};
// const t_velo_param param_1900_acc2G = {1.90f,20.0f};
// const t_velo_param param_1900_acc3G = {1.90f,30.0f};
// const t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};
// const t_straight_param st_param_1900_acc2G = {&param_1900_acc2G,&sp_gain_1900,&om_gain_1900};
// const t_straight_param st_param_1900_acc3G = {&param_1900_acc3G,&sp_gain_1900,&om_gain_1900};

// const t_pid_gain sp_gain_2000 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_2000 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_2000 = {2.0f,12.0f};
// const t_velo_param param_2000_acc2G = {2.0f,20.0f};
// const t_velo_param param_2000_acc3G = {2.0f,30.0f};
// const t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};
// const t_straight_param st_param_2000_acc2G = {&param_2000_acc2G,&sp_gain_2000,&om_gain_2000};
// const t_straight_param st_param_2000_acc3G = {&param_2000_acc3G,&sp_gain_2000,&om_gain_2000};


// const t_pid_gain sp_gain_2100 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_2100 = {0.2f, 0.01f, 0.00f};
// const t_velo_param param_2100 = {2.1f,12.0f};
// const t_velo_param param_2100_acc2G = {2.1f,20.0f};
// const t_velo_param param_2100_acc3G = {2.1f,30.0f};
// const t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};
// const t_straight_param st_param_2100_acc2G = {&param_2100_acc2G,&sp_gain_2100,&om_gain_2100};
// const t_straight_param st_param_2100_acc3G = {&param_2100_acc3G,&sp_gain_2100,&om_gain_2100};

// const t_velo_param param_2200 = {2.2f,12.0f};
// const t_velo_param param_2200_acc2G = {2.2f,20.0f};
// const t_velo_param param_2200_acc3G = {2.2f,30.0f};
// const t_straight_param st_param_2200 = make_straight_param(&param_2200,straight_control_high);
// const t_straight_param st_param_2200_acc2G = make_straight_param(&param_2200_acc2G,straight_control_high);
// const t_straight_param st_param_2200_acc3G = make_straight_param(&param_2200_acc3G,straight_control_high);

// const t_velo_param param_2300 = {2.3f,12.0f};
// const t_velo_param param_2300_acc2G = {2.3f,20.0f};
// const t_velo_param param_2300_acc3G = {2.3f,30.0f};
// const t_straight_param st_param_2300 = make_straight_param(&param_2300,straight_control_high);
// const t_straight_param st_param_2300_acc2G = make_straight_param(&param_2300_acc2G,straight_control_high);
// const t_straight_param st_param_2300_acc3G = make_straight_param(&param_2300_acc3G,straight_control_high);

// const t_velo_param param_2400 = {2.4f,12.0f};
// const t_velo_param param_2400_acc2G = {2.4f,20.0f};
// const t_velo_param param_2400_acc3G = {2.4f,30.0f};
// const t_straight_param st_param_2400 = make_straight_param(&param_2400,straight_control_high);
// const t_straight_param st_param_2400_acc2G = make_straight_param(&param_2400_acc2G,straight_control_high);
// const t_straight_param st_param_2400_acc3G = make_straight_param(&param_2400_acc3G,straight_control_high);

// const t_velo_param param_2500 = {2.5f,12.0f};
// const t_velo_param param_2500_acc2G = {2.5f,20.0f};
// const t_velo_param param_2500_acc3G = {2.5f,30.0f};
// const t_straight_param st_param_2500 = make_straight_param(&param_2500,straight_control_high);
// const t_straight_param st_param_2500_acc2G = make_straight_param(&param_2500_acc2G,straight_control_high);
// const t_straight_param st_param_2500_acc3G = make_straight_param(&param_2500_acc3G,straight_control_high);

// const t_velo_param param_2600 = {2.6f,15.0f};
// const t_velo_param param_2600_acc2G = {2.6f,20.0f};
// const t_velo_param param_2600_acc3G = {2.6f,30.0f};
// const t_straight_param st_param_2600 = make_straight_param(&param_2600,straight_control_high);
// const t_straight_param st_param_2600_acc2G = make_straight_param(&param_2600_acc2G,straight_control_high);
// const t_straight_param st_param_2600_acc3G = make_straight_param(&param_2600_acc3G,straight_control_high);

// const t_velo_param param_2700 = {2.7f,15.0f};
// const t_velo_param param_2700_acc2G = {2.7f,20.0f};
// const t_velo_param param_2700_acc3G = {2.7f,30.0f};
// const t_straight_param st_param_2700 = make_straight_param(&param_2700,straight_control_high);
// const t_straight_param st_param_2700_acc2G = make_straight_param(&param_2700_acc2G,straight_control_high);
// const t_straight_param st_param_2700_acc3G = make_straight_param(&param_2700_acc3G,straight_control_high);


// const t_velo_param param_2800 = {2.8f,15.0f};
// const t_velo_param param_2800_acc2G = {2.8f,20.0f};
// const t_velo_param param_2800_acc3G = {2.8f,30.0f};
// const t_straight_param st_param_2800 = make_straight_param(&param_2800,straight_control_high);
// const t_straight_param st_param_2800_acc2G = make_straight_param(&param_2800_acc2G,straight_control_high);
// const t_straight_param st_param_2800_acc3G = make_straight_param(&param_2800_acc3G,straight_control_high);


// const t_velo_param param_2900 = {2.9f,15.0f};
// const t_velo_param param_2900_acc2G = {2.9f,20.0f};
// const t_velo_param param_2900_acc3G = {2.9f,30.0f};
// const t_straight_param st_param_2900 = make_straight_param(&param_2900,straight_control_high);
// const t_straight_param st_param_2900_acc2G = make_straight_param(&param_2900_acc2G,straight_control_high);
// const t_straight_param st_param_2900_acc3G = make_straight_param(&param_2900_acc3G,straight_control_high);

// const t_velo_param param_3000 = {3.0f,15.0f};
// const t_velo_param param_3000_acc2G = {3.0f,20.0f};
// const t_velo_param param_3000_acc3G = {3.0f,30.0f};
// const t_straight_param st_param_3000 = make_straight_param(&param_3000,straight_control_high);
// const t_straight_param st_param_3000_acc2G = make_straight_param(&param_3000_acc2G,straight_control_high);
// const t_straight_param st_param_3000_acc3G = make_straight_param(&param_3000_acc3G,straight_control_high);

// const t_velo_param param_3200 = {3.2f,15.0f};
// const t_velo_param param_3200_acc2G = {3.2f,20.0f};
// const t_velo_param param_3200_acc3G = {3.2f,30.0f};
// const t_straight_param st_param_3200 = make_straight_param(&param_3200,straight_control_high);
// const t_straight_param st_param_3200_acc2G = make_straight_param(&param_3200_acc2G,straight_control_high);
// const t_straight_param st_param_3200_acc3G = make_straight_param(&param_3200_acc3G,straight_control_high);

// const t_velo_param param_3400 = {3.4f,15.0f};
// const t_velo_param param_3400_acc2G = {3.4f,20.0f};
// const t_velo_param param_3400_acc3G = {3.4f,30.0f};
// const t_straight_param st_param_3400 = make_straight_param(&param_3400,straight_control_high);
// const t_straight_param st_param_3400_acc2G = make_straight_param(&param_3400_acc2G,straight_control_high);
// const t_straight_param st_param_3400_acc3G = make_straight_param(&param_3400_acc3G,straight_control_high);

// const t_velo_param param_3600 = {3.6f,15.0f};
// const t_velo_param param_3600_acc2G = {3.6f,20.0f};
// const t_velo_param param_3600_acc3G = {3.6f,30.0f};
// const t_straight_param st_param_3600 = make_straight_param(&param_3600,straight_control_high);
// const t_straight_param st_param_3600_acc2G = make_straight_param(&param_3600_acc2G,straight_control_high);
// const t_straight_param st_param_3600_acc3G = make_straight_param(&param_3600_acc3G,straight_control_high);

// const t_velo_param param_3800 = {3.8f,15.0f};
// const t_velo_param param_3800_acc2G = {3.8f,20.0f};
// const t_velo_param param_3800_acc3G = {3.8f,30.0f};
// const t_straight_param st_param_3800 = make_straight_param(&param_3800,straight_control_high);
// const t_straight_param st_param_3800_acc2G = make_straight_param(&param_3800_acc2G,straight_control_high);
// const t_straight_param st_param_3800_acc3G = make_straight_param(&param_3800_acc3G,straight_control_high);

// const t_velo_param param_4000 = {4.0f,15.0f};
// const t_velo_param param_4000_acc2G = {4.0f,20.0f};
// const t_velo_param param_4000_acc3G = {4.0f,30.0f};
// const t_straight_param st_param_4000 = make_straight_param(&param_4000,straight_control_high);
// const t_straight_param st_param_4000_acc2G = make_straight_param(&param_4000_acc2G,straight_control_high);
// const t_straight_param st_param_4000_acc3G = make_straight_param(&param_4000_acc3G,straight_control_high);

// const t_velo_param param_4200 = {4.2f,15.0f};
// const t_velo_param param_4200_acc2G = {4.2f,20.0f};
// const t_velo_param param_4200_acc3G = {4.2f,30.0f};
// const t_straight_param st_param_4200 = make_straight_param(&param_4200,straight_control_high);
// const t_straight_param st_param_4200_acc2G = make_straight_param(&param_4200_acc2G,straight_control_high);
// const t_straight_param st_param_4200_acc3G = make_straight_param(&param_4200_acc3G,straight_control_high);

// const t_velo_param param_4400 = {4.4f,15.0f};
// const t_velo_param param_4400_acc2G = {4.4f,20.0f};
// const t_velo_param param_4400_acc3G = {4.4f,30.0f};
//const static t_velo_param param_4600_acc5G = {4.4f,50.0f};
// const t_straight_param st_param_4400 = make_straight_param(&param_4400,straight_control_high);
// const t_straight_param st_param_4400_acc2G = make_straight_param(&param_4400_acc2G,straight_control_high);
// const t_straight_param st_param_4400_acc3G = make_straight_param(&param_4400_acc3G,straight_control_high);

// const t_velo_param param_4600 = {4.6f,15.0f};
// const t_velo_param param_4600_acc2G = {4.6f,20.0f};
// const t_velo_param param_4600_acc3G = {4.6f,30.0f};
// const t_straight_param st_param_4600 = make_straight_param(&param_4600,straight_control_high);
// const t_straight_param st_param_4600_acc2G = make_straight_param(&param_4600_acc2G,straight_control_high);
// const t_straight_param st_param_4600_acc3G = make_straight_param(&param_4600_acc3G,straight_control_high);

// const t_velo_param param_4800 = {4.8f,15.0f};
// const t_velo_param param_4800_acc2G = {4.8f,20.0f};
// const t_velo_param param_4800_acc3G = {4.8f,30.0f};
// const t_straight_param st_param_4800 = make_straight_param(&param_4800,straight_control_high);
// const t_straight_param st_param_4800_acc2G = make_straight_param(&param_4800_acc2G,straight_control_high);
// const t_straight_param st_param_4800_acc3G = make_straight_param(&param_4800_acc3G,straight_control_high);


// const t_velo_param param_5000 = {5.0f,15.0f};
// const t_velo_param param_5000_acc2G = {5.0f,20.0f};
// const t_velo_param param_5000_acc3G = {5.0f,30.0f};
// const t_straight_param st_param_5000 = make_straight_param(&param_5000,straight_control_high);
// const t_straight_param st_param_5000_acc2G = make_straight_param(&param_5000_acc2G,straight_control_high);
// const t_straight_param st_param_5000_acc3G = make_straight_param(&param_5000_acc3G,straight_control_high);


// const t_velo_param param_1400_acc5G = {1.40f, 50.0f};
// const t_straight_param st_param_1400_acc5G = {&param_1400_acc5G, &sp_gain_1400, &om_gain_1400};

// const t_velo_param param_1500_acc5G = {1.50f, 50.0f};
// const t_straight_param st_param_1500_acc5G = {&param_1500_acc5G, &sp_gain_1500, &om_gain_1500};

// const t_velo_param param_1600_acc5G = {1.60f, 50.0f};
// const t_straight_param st_param_1600_acc5G = {&param_1600_acc5G, &sp_gain_1600, &om_gain_1600};

// const t_velo_param param_1700_acc5G = {1.70f, 50.0f};
// const t_straight_param st_param_1700_acc5G = {&param_1700_acc5G, &sp_gain_1700, &om_gain_1700};

// const t_velo_param param_1800_acc5G = {1.80f, 50.0f};
// const t_straight_param st_param_1800_acc5G = {&param_1800_acc5G, &sp_gain_1800, &om_gain_1800};

// const t_velo_param param_1900_acc5G = {1.90f, 50.0f};
// const t_straight_param st_param_1900_acc5G = {&param_1900_acc5G, &sp_gain_1900, &om_gain_1900};

// const t_velo_param param_2000_acc5G = {2.00f, 50.0f};
// const t_straight_param st_param_2000_acc5G = {&param_2000_acc5G, &sp_gain_2000, &om_gain_2000};

// const t_velo_param param_2100_acc5G = {2.10f, 50.0f};
// const t_straight_param st_param_2100_acc5G = {&param_2100_acc5G, &sp_gain_2100, &om_gain_2100};

// const t_velo_param param_2200_acc5G = {2.20f, 50.0f};
// const t_straight_param st_param_2200_acc5G = make_straight_param(&param_2200_acc5G,straight_control_high);

// const t_velo_param param_2300_acc5G = {2.30f, 50.0f};
// const t_straight_param st_param_2300_acc5G = make_straight_param(&param_2300_acc5G,straight_control_high);

// const t_velo_param param_2400_acc5G = {2.40f, 50.0f};
// const t_straight_param st_param_2400_acc5G = make_straight_param(&param_2400_acc5G,straight_control_high);

// const t_velo_param param_2500_acc5G = {2.50f, 50.0f};
// const t_straight_param st_param_2500_acc5G = make_straight_param(&param_2500_acc5G,straight_control_high);

// const t_velo_param param_2600_acc5G = {2.60f, 50.0f};
// const t_straight_param st_param_2600_acc5G = make_straight_param(&param_2600_acc5G,straight_control_high);

// const t_velo_param param_2700_acc5G = {2.70f, 50.0f};
// const t_straight_param st_param_2700_acc5G = make_straight_param(&param_2700_acc5G,straight_control_high);

// const t_velo_param param_2800_acc5G = {2.80f, 50.0f};
// const t_straight_param st_param_2800_acc5G = make_straight_param(&param_2800_acc5G,straight_control_high);

// const t_velo_param param_2900_acc5G = {2.90f, 50.0f};
// const t_straight_param st_param_2900_acc5G = make_straight_param(&param_2900_acc5G,straight_control_high);

// const t_velo_param param_3000_acc5G = {3.00f, 50.0f};
// const t_straight_param st_param_3000_acc5G = make_straight_param(&param_3000_acc5G,straight_control_high);

// const t_velo_param param_3200_acc5G = {3.20f, 50.0f};
// const t_straight_param st_param_3200_acc5G = make_straight_param(&param_3200_acc5G,straight_control_high);

// const t_velo_param param_3400_acc5G = {3.40f, 50.0f};
// const t_straight_param st_param_3400_acc5G = make_straight_param(&param_3400_acc5G,straight_control_high);

// const t_velo_param param_3600_acc5G = {3.60f, 50.0f};
// const t_straight_param st_param_3600_acc5G = make_straight_param(&param_3600_acc5G,straight_control_high);

// const t_velo_param param_3800_acc5G = {3.80f, 50.0f};
// const t_straight_param st_param_3800_acc5G = make_straight_param(&param_3800_acc5G,straight_control_high);

// const t_velo_param param_4000_acc5G = {4.00f, 50.0f};
// const t_straight_param st_param_4000_acc5G = make_straight_param(&param_4000_acc5G,straight_control_high);

// const t_velo_param param_4200_acc5G = {4.20f, 50.0f};
// const t_straight_param st_param_4200_acc5G = make_straight_param(&param_4200_acc5G,straight_control_high);

// const t_velo_param param_4400_acc5G = {4.40f, 50.0f};
// const t_straight_param st_param_4400_acc5G = make_straight_param(&param_4400_acc5G,straight_control_high);

// const t_velo_param param_4600_acc5G = {4.60f, 50.0f};
// const t_straight_param st_param_4600_acc5G = make_straight_param(&param_4600_acc5G,straight_control_high);





// 旧候補配列（参照廃止）。速度候補はt_speed_profile、PID/FFはt_control_profileを使う。
// const t_straight_param *const st_mode_300_v0[] = {&st_param_300};
// const t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
// const t_straight_param *const st_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
// const t_straight_param *const st_mode_700_v0[] = {&st_param_700};
// const t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
// const t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
// const t_straight_param *const st_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
// const t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };

// const t_straight_param *const st_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };

// const t_straight_param *const st_mode_1400_v1[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,&st_param_3200,
	// &st_param_3400,
// };
// const t_straight_param *const st_mode_1400_v2[] = {&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };
// const t_straight_param *const st_mode_1400_v3[] = {&st_param_1400_acc3G,&st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };



// const t_straight_param *const st_mode_1500_v0[] = {&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };
// const t_straight_param *const st_mode_1500_v1[] = {&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,
// };

// const t_straight_param *const st_mode_1500_v2[] = {&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };


// const t_straight_param *const st_mode_1600_v1[] = {&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };

// const t_straight_param *const st_mode_1600_v2[] = {&st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };

// const t_straight_param *const st_mode_1600_v3[] = {&st_param_1600_acc5G,&st_param_1800_acc5G,&st_param_2000_acc5G,
	// &st_param_2200_acc5G,&st_param_2400_acc5G,&st_param_2600_acc5G,&st_param_2800_acc5G,&st_param_3000_acc5G,&st_param_3200_acc5G,
	// &st_param_3400_acc5G,&st_param_3600_acc5G,&st_param_3800_acc5G,&st_param_4000_acc5G,&st_param_4200_acc5G,&st_param_4400_acc5G,
	// &st_param_4600_acc5G,
// };
// const t_straight_param *const st_mode_1800_v1[] = {&st_param_1800_acc3G,&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };
// const t_straight_param *const st_mode_2000_v1[] = {&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };

// const t_straight_param *const di_mode_300_v0[] = {&st_param_300};
// const t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
// const t_straight_param *const di_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
// const t_straight_param *const di_mode_700_v0[] = {&st_param_700};
// const t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};
// const t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
// const t_straight_param *const di_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
// const t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };
// const t_straight_param *const di_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
// const t_straight_param *const di_mode_1400_v1[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };
// const t_straight_param *const di_mode_1400_v2[] =  {&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };

// const t_straight_param *const di_mode_1500_v0[] = {&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
	// &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000,
// };
// const t_straight_param *const di_mode_1500_v1[] = {&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,
// };
// const t_straight_param *const di_mode_1500_v2[] =  {&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };

// const t_straight_param *const di_mode_1600_v1[] =  {&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };

// const t_straight_param *const di_mode_1600_v2[] = {&st_param_1600_acc3G,&st_param_1800_acc3G,&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };


// const t_straight_param *const di_mode_1800_v1[] =  {&st_param_1800_acc2G,&st_param_2000_acc2G,
	// &st_param_2200_acc2G,&st_param_2400_acc2G,&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,
	// &st_param_3400_acc2G,&st_param_3600_acc2G,&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,
	// &st_param_4600_acc2G,
// };

// const t_straight_param *const di_mode_2000_v1[] = {&st_param_2000_acc3G,
	// &st_param_2200_acc3G,&st_param_2400_acc3G,&st_param_2600_acc3G,&st_param_2800_acc3G,&st_param_3000_acc3G,&st_param_3200_acc3G,
	// &st_param_3400_acc3G,&st_param_3600_acc3G,&st_param_3800_acc3G,&st_param_4000_acc3G,&st_param_4200_acc3G,&st_param_4400_acc3G,
	// &st_param_4600_acc3G,
// };

const t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,Turn_L};
const t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};


const t_param *const *const acc_mode_1400[] = {mode_1400_acc,mode_1600_acc_v1};
const t_param *const *const acc_mode_1600_v1[] = {mode_1600_acc_v2,mode_1800_acc_v1};
const t_param *const *const acc_mode_1600_v2[] = {mode_1600_acc_v2,mode_1800_acc_v1};
const t_param *const *const acc_mode_1600_v3[] = {mode_1600_acc_v2,mode_1800_acc_v2,mode_2000_acc};
const t_param *const *const acc_mode_1800_v1[] = {mode_1800,mode_2000_acc_v2};

