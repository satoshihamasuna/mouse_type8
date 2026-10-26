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
#ifndef ZIRCONIA_WALL_PARAM_H_
#define ZIRCONIA_WALL_PARAM_H_

// Initial values from program_zirconia's independent_side_pose_v3.
// SI units internally; these are not yet tuned on Mouse A or Mouse B.
namespace ZirconiaWallParams {
constexpr float reference_m = 0.045f;
constexpr float found_m = 0.065f;
constexpr float lost_m = 0.070f;
constexpr float derivative_window_m = 0.003f;
constexpr float recovery_m = 0.006f;
constexpr float residual_reject = 0.35f;
constexpr float residual_accept = 0.18f;
constexpr float signal_slope_reject_per_m = 8.0f;
constexpr float confirm_m = 0.003f;
constexpr float away_only_m = 0.027f;
constexpr float transition_m = 0.010f;
constexpr float trusted_error_m = 0.010f;
constexpr float distance_kp = 5000.0f;
constexpr float omega_kd = 50.0f;
constexpr float velocity_damping = 55.0f;
constexpr float alpha_limit = 4.0f;
constexpr float omega_limit = 0.2f;
constexpr float tracking_distance_komega = 300.0f;
constexpr float tracking_heading_komega = 20.0f;
constexpr float tracking_motion_scale = 1.7f;
constexpr float tracking_alpha_limit = 20.0f;
constexpr float tracking_omega_limit = 1.2f;
constexpr float measurement_blend_m = 0.010f;
constexpr float sync_error_m = 0.005f;
constexpr float sync_slope = 0.10f;
constexpr float sync_confirm_m = 0.003f;
}
#endif
