/* Copyright 2026 hmsn
 * Modifications Copyright 2026 hmsn
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* 機体Bの壁制御設定。実機同定前の初期値。距離校正はsens_table_B.hを使用する。 */
#ifndef WALL_PARAM_B_H_
#define WALL_PARAM_B_H_

#include "Params/typedef_wall_param.h"

// zirconia移植元の初期値。距離[m]、速度[m/s]、姿勢[rad]。
const static t_wall_control_param wall_control_param = {
	0.045f,	// reference_m
	0.065f,	// found_m
	0.070f,	// lost_m
	0.003f,	// derivative_window_m
	0.006f,	// recovery_m
	0.35f,	// residual_reject
	0.18f,	// residual_accept
	8.0f,	// signal_slope_reject_per_m
	0.003f,	// confirm_m
	0.027f,	// away_only_m
	0.010f,	// transition_m
	0.010f,	// trusted_error_m
	5000.0f,	// distance_kp
	50.0f,	// omega_kd
	55.0f,	// velocity_damping
	4.0f,	// alpha_limit
	0.2f,	// omega_limit
	300.0f,	// tracking_distance_komega
	20.0f,	// tracking_heading_komega
	1.7f,	// tracking_motion_scale
	20.0f,	// tracking_alpha_limit
	1.2f,	// tracking_omega_limit
	0.010f,	// measurement_blend_m
	0.005f,	// sync_error_m
	0.10f,	// sync_slope
	0.003f,	// sync_confirm_m
};

// 2状態EKF。距離[m]、姿勢[rad]、分散[m²/rad²]、時間[s]。
const static t_wall_ekf_param wall_ekf_param = {
	0.012f,	// initial_lateral_std_m
	0.13962634f,	// initial_heading_std_rad
	0.002f,	// lateral_walk_m_sqrt_s
	0.005235988f,	// heading_walk_rad_sqrt_s
	0.0000015625f,	// side_variance_m2
	0.0000010625f,	// both_variance_m2
	0.000009f,	// corner_variance_m2
	0.020f,	// innovation_limit_m
	9.0f,	// nis_limit
	0.010f,	// observation_interval_s
	0.005f,	// lateral_std_limit_m
	0.05235988f,	// heading_std_limit_rad
	0.2617994f,	// heading_mean_limit_rad
	0.030f,	// lateral_mean_limit_m
	0.090f,	// coast_limit_m
	0.008f,	// edge_guard_m
	0.05235988f,	// heading_disagreement_limit_rad: 区間ジャイロ姿勢との差3度（未実機評価）
};

// 機体Bの斜めEKF試作値。校正はBを使用し、実機調整は未実施。
const static t_diagonal_wall_param diagonal_wall_param = {
	0.032f,	// reference_m
	0.025f,	// minimum_range_m
	0.003f,	// confirmation_m
	0.003f,	// observation_std_m
	0.066f,	// front_block_m: 両前壁が近いときだけ柱観測を除外
};

// 壁切れ。距離[mm]、角速度[rad/s]、ticksは制御回数。
const static t_wall_edge_param wall_edge_param = {
	65.0f,	// crossing_mm
	65.0f,	// arm_range_mm
	0.5f,	// arm_travel_mm
	66.0f,	// confirm_range_mm
	0.5f,	// confirm_travel_mm
	8.0f,	// maximum_pending_mm
	32U,	// maximum_pending_ticks
	4U,	// recent_forward_ticks
	6.0f,	// maximum_sample_step_mm
	2.0f,	// maximum_yaw_rate_radps
};

// Bの斜め柱通過の試作値。Aの実測位置校正を転用していない。
const static t_diagonal_edge_param diagonal_edge_param = {
	1.0f,	// sample_step_mm
	60.0f,	// near_mm
	0.5f,	// arm_difference_mm
	0.5f,	// arm_travel_mm
	0.5f,	// confirm_difference_mm
	1.0f,	// confirm_rise_mm
	0.25f,	// rebound_mm
	0.5f,	// confirm_travel_mm
	65.0f,	// release_mm
	1.0f,	// release_travel_mm
	5.0f,	// release_rise_mm
	20.0f,	// rearm_travel_mm
	8.0f,	// maximum_pending_mm
	32U,	// maximum_pending_ticks
	4U,	// recent_forward_ticks
	6.0f,	// maximum_sample_step_mm
	2.0f,	// maximum_yaw_rate_radps
};

// 探索ターンの補正判定。距離[mm]、姿勢[rad]、位置分散[mm²]。
const static t_search_turn_guard_param search_turn_guard_param = {
	15.0f,	// side_error_mm
	10.0f,	// front_difference_mm
	8.0f,	// front_release_mm
	20.0f,	// hard_front_difference_mm
	80.0f,	// front_near_mm
	55.0f,	// front_valid_min_mm
	115.0f,	// front_valid_max_mm
	3.0f,	// confirmation_mm
	3.0f,	// pair_change_difference_mm
	4.0f,	// range_change_tolerance_mm
	5.0f,	// centered_mm
	4.0f,	// lateral_variance_limit_mm2
	0.034906585f,	// aligned_rad
	0.5f,	// quiet_yaw_radps
	0.2617994f,	// exit_residual_limit_rad
};

#endif /* WALL_PARAM_B_H_ */
