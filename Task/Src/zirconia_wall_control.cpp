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
#include "Task/Inc/zirconia_wall_control.h"
#include <math.h>

namespace {
float limit(float value, float bound) { return fmaxf(-bound, fminf(bound, value)); }
}

void ZirconiaWallControl::reset() { *this = ZirconiaWallControl{}; }

void ZirconiaWallControl::prime_tracking(float left_m, float right_m)
{
    reset();
    if (!isfinite(left_m) || !isfinite(right_m)) return;
    left_detected_ = left_m <= ZirconiaWallParams::found_m;
    right_detected_ = right_m <= ZirconiaWallParams::found_m;
    observed_mask_ = (left_detected_ ? 1U : 0U) | (right_detected_ ? 2U : 0U);
    phase_ = observed_mask_ ? Tracking : Estimated;
}

void ZirconiaWallControl::Side::update(bool available, float distance,
    float signal_value, float travel, float predicted, bool sample_updated)
{
    using namespace ZirconiaWallParams;
    if (!available) { *this = Side{}; return; }
    travel_m += fabsf(travel);
    predicted_m += predicted;
    if (!sample_updated || (anchor && travel_m < derivative_window_m)) return;
    if (anchor) {
        const float residual = fabsf((distance - distance_m - predicted_m) / travel_m);
        const float signal_slope = (signal_value - signal) / (fmaxf(100.0f, signal) * travel_m);
        if (residual > residual_reject
            || (residual > residual_accept && fabsf(signal_slope) > signal_slope_reject_per_m)) {
            stable = false;
            recovery_m = 0.0f;
        } else if (residual < residual_accept) {
            recovery_m += travel_m;
            if (recovery_m >= ZirconiaWallParams::recovery_m) stable = true;
        } else {
            recovery_m = 0.0f;
        }
    }
    anchor = true;
    distance_m = distance;
    signal = signal_value;
    travel_m = predicted_m = 0.0f;
}

ZirconiaWallControl::Output ZirconiaWallControl::update(const Input &in)
{
    using namespace ZirconiaWallParams;
    Output out;
    if (!in.enabled || !isfinite(in.left_m) || !isfinite(in.right_m)
        || !isfinite(in.left_signal) || !isfinite(in.right_signal)
        || !isfinite(in.travel_m) || !isfinite(in.velocity_mps)
        || !isfinite(in.heading_rad) || !isfinite(in.estimated_error_m)
        || !isfinite(in.base_omega_radps) || !isfinite(in.dt_s) || in.dt_s <= 0.0f) {
        reset();
        return out;
    }
    if (in.sample_updated) {
        left_detected_ = in.left_m <= (left_detected_ ? lost_m : found_m);
        right_detected_ = in.right_m <= (right_detected_ ? lost_m : found_m);
    }
    const float ds = fabsf(in.travel_m);
    const float predicted = in.travel_m * sinf(in.heading_rad) * tracking_motion_scale;
    left_.update(left_detected_, in.left_m, in.left_signal, in.travel_m, -predicted, in.sample_updated);
    right_.update(right_detected_, in.right_m, in.right_signal, in.travel_m, predicted, in.sample_updated);
    const uint8_t mask = (left_detected_ ? 1U : 0U) | (right_detected_ ? 2U : 0U);
    const uint8_t trusted = (left_.stable ? 1U : 0U) | (right_.stable ? 2U : 0U);
    const uint8_t previous_mask = observed_mask_;
    if (phase_ == Tracking && (mask & previous_mask & trusted) != 0) {
        observed_mask_ = mask;
        phase_distance_m_ += ds;
        transition_ = fminf(1.0f, transition_ + ds / transition_m);
    } else if (phase_ == Disabled || mask != observed_mask_) {
        observed_mask_ = mask;
        phase_ = mask ? Confirming : Estimated;
        phase_distance_m_ = transition_ = 0.0f;
        tracking_valid_ = false;
    } else {
        phase_distance_m_ += ds;
        if (phase_ == Confirming) {
            if (phase_distance_m_ >= confirm_m) {
                phase_ = AwayOnly;
                phase_distance_m_ = transition_ = 0.0f;
            }
        } else {
            transition_ = fminf(1.0f, transition_ + ds / transition_m);
            if (phase_ == AwayOnly && (trusted || phase_distance_m_ >= away_only_m)) {
                phase_ = Tracking;
                phase_distance_m_ = transition_ = 0.0f;
            }
        }
    }
    if ((previous_mask & static_cast<uint8_t>(~mask)) != 0
        && (previous_mask & mask & trusted) == 0) omega_ = 0.0f;

    const float left_error = in.left_m - reference_m;
    const float right_error = reference_m - in.right_m;
    const uint8_t error_mask = trusted ? trusted : mask;
    float raw_error = error_mask == 3 ? 0.5f * (left_error + right_error)
        : error_mask == 1 ? left_error : error_mask == 2 ? right_error : 0.0f;
    const bool measurement_trusted = phase_ == Tracking && fabsf(raw_error) <= trusted_error_m;
    const bool pose_control = phase_ == Tracking && mask
        && (trusted || tracking_valid_ || mask == 3);
    float error = raw_error;
    if (phase_ == Estimated) error = in.estimated_error_m;
    else if (phase_ == Confirming) error = 0.0f;
    else if (pose_control) {
        if (!tracking_valid_) {
            tracking_error_m_ = trusted ? raw_error : 0.0f;
            tracking_valid_ = trusted != 0;
        }
        tracking_error_m_ -= predicted;
        if (trusted) {
            tracking_error_m_ += fminf(1.0f, ds / measurement_blend_m)
                * (raw_error - tracking_error_m_);
        }
        tracking_error_m_ = limit(tracking_error_m_, reference_m);
        error = tracking_error_m_;
    } else if (phase_ == AwayOnly || !measurement_trusted) {
        const float left_away = (mask & 1) ? fminf(0.0f, left_error) : 0.0f;
        const float right_away = (mask & 2) ? fmaxf(0.0f, right_error) : 0.0f;
        error = fabsf(left_away) >= fabsf(right_away) ? left_away : right_away;
    }

    float step;
    if (pose_control) {
        const float desired_raw = transition_
            * (tracking_distance_komega * error - tracking_heading_komega * in.heading_rad);
        const float desired = limit(desired_raw, tracking_omega_limit);
        step = limit(desired - omega_, tracking_alpha_limit * in.dt_s);
        out.alpha_saturated = fabsf(desired - omega_) > tracking_alpha_limit * in.dt_s;
        out.omega_saturated = desired != desired_raw;
    } else {
        const float alpha_raw = distance_kp * error * transition_
            - (omega_kd + velocity_damping * fabsf(in.velocity_mps)) * (in.base_omega_radps + omega_);
        const float alpha = limit(alpha_raw, alpha_limit);
        const float candidate = omega_ + alpha * in.dt_s;
        const float desired = limit(candidate, omega_limit);
        step = limit(desired - omega_, alpha_limit * in.dt_s);
        out.alpha_saturated = alpha_raw != alpha;
        out.omega_saturated = omega_ + step != candidate;
    }
    omega_ += step;
    out.error_m = error;
    out.raw_error_m = raw_error;
    out.omega_radps = omega_;
    out.alpha_radps2 = step / in.dt_s;
    out.transition = transition_;
    out.phase_distance_m = phase_distance_m_;
    out.observed_mask = mask;
    out.trusted_mask = trusted;
    out.phase = phase_;
    out.pose_control = pose_control;

    // Conservative synchronization of the caller's lateral estimate; the
    // heading used for damping is deliberately not changed by this operation.
    sync_travel_m_ += ds;
    if (!measurement_trusted || !trusted || transition_ < 1.0f || fabsf(raw_error) > sync_error_m) {
        sync_valid_ = false;
        sync_stable_m_ = sync_travel_m_ = 0.0f;
    } else if (in.sample_updated) {
        const bool stable = sync_valid_ && mask == sync_mask_ && sync_travel_m_ > 0.0f
            && fabsf(raw_error - sync_error_m_) <= sync_travel_m_ * sync_slope;
        sync_stable_m_ = stable ? sync_stable_m_ + sync_travel_m_ : 0.0f;
        sync_error_m_ = raw_error;
        sync_mask_ = mask;
        sync_valid_ = true;
        sync_travel_m_ = 0.0f;
    }
    out.sync_estimator = sync_valid_ && sync_stable_m_ >= sync_confirm_m;
    return out;
}
