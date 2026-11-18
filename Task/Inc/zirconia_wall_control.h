#ifndef TASK_INC_ZIRCONIA_WALL_CONTROL_H_
#define TASK_INC_ZIRCONIA_WALL_CONTROL_H_

#include <stdint.h>
#include "Params/wall_param.h"
#include "Task/Inc/straight_wall_ekf.h"

// 直進壁制御・ADC平均
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


// Adapted from program_zirconia/zirconia/software/Control/Src/wall_control.c
// (independent_side_pose_v3). No hardware or trajectory ownership here.
class ZirconiaWallControl
{
public:
    enum Phase : uint8_t { Disabled, Estimated, Confirming, AwayOnly, Tracking };
    struct Input {
        float left_m = 0.080f, right_m = 0.080f;
        float left_signal = 0.0f, right_signal = 0.0f;
        float travel_m = 0.0f, velocity_mps = 0.0f;
        float heading_rad = 0.0f, estimated_error_m = 0.0f;
        float base_omega_radps = 0.0f, dt_s = 0.001f;
        bool enabled = false, sample_updated = false;
        // Optional externally estimated pose. Defaults preserve legacy callers.
        bool use_estimated_pose = false, prediction_trusted = false;
        bool heading_only_gap = false;
        uint8_t permitted_mask = 3, measurement_mask = 3, range_valid_mask = 3;
    };
    struct Output {
        float error_m = 0.0f, raw_error_m = 0.0f;
        float omega_radps = 0.0f, alpha_radps2 = 0.0f;
        float transition = 0.0f, phase_distance_m = 0.0f;
        uint8_t observed_mask = 0, trusted_mask = 0;
        Phase phase = Disabled;
        bool pose_control = false, sync_estimator = false;
        bool alpha_saturated = false, omega_saturated = false;
    };
    void reset();
    void prime_tracking(float left_m, float right_m);
    Output update(const Input &input);

private:
    struct Side {
        float distance_m = 0.0f, signal = 0.0f;
        float travel_m = 0.0f, predicted_m = 0.0f, recovery_m = 0.0f;
        bool anchor = false, stable = false;
        void update(bool available, float distance, float signal_value,
                    float travel, float predicted, bool sample_updated);
    };
    Side left_, right_;
    Phase phase_ = Disabled;
    uint8_t observed_mask_ = 0;
    bool left_detected_ = false, right_detected_ = false;
    float phase_distance_m_ = 0.0f, transition_ = 0.0f, omega_ = 0.0f;
    float tracking_error_m_ = 0.0f;
    bool tracking_valid_ = false;
    float sync_error_m_ = 0.0f, sync_travel_m_ = 0.0f, sync_stable_m_ = 0.0f;
    uint8_t sync_mask_ = 0;
    bool sync_valid_ = false;
};

// Match zirconia's ten fresh ADC readings, independently of the legacy
// twenty-cycle distance history used by wall-edge detection.
class ZirconiaWallAverage
{
public:
    void update(int16_t value) {
        sum_ -= samples_[index_];
        samples_[index_] = value > 0 ? value : 0;
        sum_ += samples_[index_];
        index_ = (index_ + 1) % 10;
        if (count_ < 10) ++count_;
    }
    bool ready() const { return count_ == 10; }
    int16_t value() const { return count_ ? static_cast<int16_t>(sum_ / count_) : 0; }
private:
    int16_t samples_[10] = {};
    int32_t sum_ = 0;
    int index_ = 0, count_ = 0;
};

// 斜め壁制御
// Local diagonal-axis e [m], psi [rad], both positive left. The calibrated
// 32mm rule is a proximity approximation around pillars, not a corridor width.
class DiagonalWallControl {
public:
    struct Input {
        float left_m = .080f, right_m = .080f;
        float front_left_m = .125f, front_right_m = .125f;
        float travel_m = 0.0f, gyro_radps = 0.0f, dt_s = .001f;
        uint8_t eligible_mask = 0;
        bool enabled = false, fresh = false;
    };
    struct Output {
        StraightWallEkf::Estimate estimate;
        float omega_radps = 0.0f, alpha_radps2 = 0.0f, error_m = 0.0f;
        uint8_t candidate_mask = 0, accepted_mask = 0;
        bool active = false, used_ekf = false, updated = false, front_blocked = false;
    };
    void reset() { filter_.clear(); left_travel_ = right_travel_ = 0.0f; previous_mask_ = 0; heading_primed_=false; }
    void prime_heading(float heading_rad, float variance_rad2) {
        reset(); filter_.start(0.0f,heading_rad,variance_rad2); heading_primed_=true;
    }
    Output update(const Input &in) {
        Output out;
        if (!in.enabled || !isfinite(in.left_m) || !isfinite(in.right_m) || !isfinite(in.travel_m)
            || !isfinite(in.front_left_m) || !isfinite(in.front_right_m)
            || !isfinite(in.gyro_radps) || !isfinite(in.dt_s) || in.dt_s <= 0.0f || in.dt_s >= .020f
            || in.travel_m < 0.0f || in.travel_m > wall_edge_param.maximum_sample_step_mm / 1000.0f) {
            reset(); return out;
        }
        if (!filter_.estimate().initialized) filter_.start(0.0f, 0.0f);
        else if (!heading_primed_) filter_.predict(in.fresh ? in.travel_m : 0.0f, in.gyro_radps * in.dt_s, in.dt_s);
        heading_primed_=false; // The handoff already includes the entry IRQ.
        uint8_t mask = 0;
        if ((in.eligible_mask & 1) && in.left_m >= diagonal_wall_param.minimum_range_m
            && in.left_m < diagonal_wall_param.reference_m) mask |= 1;
        if ((in.eligible_mask & 2) && in.right_m >= diagonal_wall_param.minimum_range_m
            && in.right_m < diagonal_wall_param.reference_m) mask |= 2;
        // A single front return may be the same diagonal pillar. Do not use
        // the straight-side gate here; retain exclusion for a close front pair.
        if (in.front_left_m <= diagonal_wall_param.front_block_m
            && in.front_right_m <= diagonal_wall_param.front_block_m) {
            out.front_blocked = mask != 0;
            mask = 0;
        }
        if (fabsf(in.gyro_radps) > wall_edge_param.maximum_yaw_rate_radps) mask = 0;
        // The first close reading does not confirm travel that preceded it.
        if (!(mask & 1)) left_travel_ = 0.0f;
        else if (in.fresh && (previous_mask_ & 1)) left_travel_ = fminf(diagonal_wall_param.confirmation_m, left_travel_ + in.travel_m);
        if (!(mask & 2)) right_travel_ = 0.0f;
        else if (in.fresh && (previous_mask_ & 2)) right_travel_ = fminf(diagonal_wall_param.confirmation_m, right_travel_ + in.travel_m);
        previous_mask_ = mask;
        if ((mask & 1) && left_travel_ >= diagonal_wall_param.confirmation_m) out.candidate_mask |= 1;
        if ((mask & 2) && right_travel_ >= diagonal_wall_param.confirmation_m) out.candidate_mask |= 2;
        const float left = diagonal_wall_param.reference_m - in.left_m;
        const float right = in.right_m - diagonal_wall_param.reference_m;
        const float raw = mask == 3 ? .5f * (left + right) : mask == 1 ? left : mask == 2 ? right : 0.0f;
        const float observation = out.candidate_mask == 3 ? .5f * (left + right)
            : out.candidate_mask == 1 ? left : right;
        const uint32_t before = filter_.estimate().wall_updates;
        out.accepted_mask = filter_.observe_proximity(observation,
            diagonal_wall_param.observation_std_m * diagonal_wall_param.observation_std_m,
            out.candidate_mask == mask ? out.candidate_mask : 0, in.fresh && in.travel_m > 0.0f);
        out.estimate = filter_.estimate();
        out.active = out.estimate.initialized;
        out.updated = out.estimate.wall_updates != before;
        out.used_ekf = out.accepted_mask != 0 && filter_.lateral_trusted();
        float error = out.used_ekf ? out.estimate.lateral_m : raw;
        if (out.used_ekf && error * raw <= 0.0f && error != raw) { error = raw; out.used_ekf = false; }
        // No pillar is not an observation of centre zero. The IR task retains
        // the existing diagonal trajectory controller until this is usable.
        if (!mask) error = 0.0f;
        // Applied omega/alpha are filled by the IR task after the shared
        // trajectory law; relative gyro yaw is not an absolute heading anchor.
        out.error_m = error;
        return out;
    }
private:
    StraightWallEkf filter_;
    float left_travel_ = 0.0f, right_travel_ = 0.0f;
    uint8_t previous_mask_ = 0;
    bool heading_primed_=false;
};

#endif
