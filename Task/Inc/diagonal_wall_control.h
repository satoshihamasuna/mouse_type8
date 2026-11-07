#ifndef DIAGONAL_WALL_CONTROL_H_
#define DIAGONAL_WALL_CONTROL_H_

#include "Task/Inc/straight_wall_ekf.h"

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
    void reset() { filter_.clear(); left_travel_ = right_travel_ = 0.0f; previous_mask_ = 0; }
    Output update(const Input &in) {
        Output out;
        if (!in.enabled || !isfinite(in.left_m) || !isfinite(in.right_m) || !isfinite(in.travel_m)
            || !isfinite(in.front_left_m) || !isfinite(in.front_right_m)
            || !isfinite(in.gyro_radps) || !isfinite(in.dt_s) || in.dt_s <= 0.0f || in.dt_s >= .020f
            || in.travel_m < 0.0f || in.travel_m > wall_edge_param.maximum_sample_step_mm / 1000.0f) {
            reset(); return out;
        }
        if (!filter_.estimate().initialized) filter_.start(0.0f, 0.0f);
        else filter_.predict(in.fresh ? in.travel_m : 0.0f, in.gyro_radps * in.dt_s, in.dt_s);
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
};
#endif
