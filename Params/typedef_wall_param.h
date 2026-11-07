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
/* 壁制御・推定・検出の設定型。値はwall_param_A/B.hに定義する。 */
#ifndef PARAMS_TYPEDEF_WALL_PARAM_H_
#define PARAMS_TYPEDEF_WALL_PARAM_H_

#include <stdint.h>

typedef struct{
	float reference_m;
	float found_m;
	float lost_m;
	float derivative_window_m;
	float recovery_m;
	float residual_reject;
	float residual_accept;
	float signal_slope_reject_per_m;
	float confirm_m;
	float away_only_m;
	float transition_m;
	float trusted_error_m;
	float distance_kp;
	float omega_kd;
	float velocity_damping;
	float alpha_limit;
	float omega_limit;
	float tracking_distance_komega;
	float tracking_heading_komega;
	float tracking_motion_scale;
	float tracking_alpha_limit;
	float tracking_omega_limit;
	float measurement_blend_m;
	float sync_error_m;
	float sync_slope;
	float sync_confirm_m;
}t_wall_control_param;

typedef struct{
	float initial_lateral_std_m;
	float initial_heading_std_rad;
	float lateral_walk_m_sqrt_s;
	float heading_walk_rad_sqrt_s;
	float side_variance_m2;
	float both_variance_m2;
	float corner_variance_m2;
	float innovation_limit_m;
	float nis_limit;
	float observation_interval_s;
	float lateral_std_limit_m;
	float heading_std_limit_rad;
	float heading_mean_limit_rad;
	float lateral_mean_limit_m;
	float coast_limit_m;
	float edge_guard_m;
	float heading_disagreement_limit_rad;
}t_wall_ekf_param;

// 斜めは断続的な柱への接近を観測する。距離[m]、姿勢[rad]。
typedef struct{
	float reference_m;
	float minimum_range_m;
	float confirmation_m;
	float observation_std_m;
	float front_block_m;
}t_diagonal_wall_param;

typedef struct{
	float crossing_mm;
	float arm_range_mm;
	float arm_travel_mm;
	float confirm_range_mm;
	float confirm_travel_mm;
	float maximum_pending_mm;
	uint16_t maximum_pending_ticks;
	uint16_t recent_forward_ticks;
	float maximum_sample_step_mm;
	float maximum_yaw_rate_radps;
}t_wall_edge_param;

typedef struct{
	float sample_step_mm;
	float near_mm;
	float arm_difference_mm;
	float arm_travel_mm;
	float confirm_difference_mm;
	float confirm_rise_mm;
	float rebound_mm;
	float confirm_travel_mm;
	float release_mm;
	float release_travel_mm;
	float release_rise_mm;
	float rearm_travel_mm;
	float maximum_pending_mm;
	uint16_t maximum_pending_ticks;
	uint16_t recent_forward_ticks;
	float maximum_sample_step_mm;
	float maximum_yaw_rate_radps;
}t_diagonal_edge_param;

typedef struct{
	float side_error_mm;
	float front_difference_mm;
	float front_release_mm;
	float hard_front_difference_mm;
	float front_near_mm;
	float front_valid_min_mm;
	float front_valid_max_mm;
	float confirmation_mm;
	float pair_change_difference_mm;
	float range_change_tolerance_mm;
	float centered_mm;
	float lateral_variance_limit_mm2;
	float aligned_rad;
	float quiet_yaw_radps;
	float exit_residual_limit_rad;
}t_search_turn_guard_param;

#endif /* PARAMS_TYPEDEF_WALL_PARAM_H_ */
