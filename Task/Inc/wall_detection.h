#ifndef TASK_INC_WALL_DETECTION_H_
#define TASK_INC_WALL_DETECTION_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

// 側壁観測の安定性
// Resample the side-wall distance every 1 mm of forward encoder travel.
// The 6 mm window approximates the old 20 ms window at 0.32 m/s.
class SideWallStability
{
public:
    static constexpr float kSampleStepMm = 1.0f;
    static constexpr float kDifferenceLimitMm = 5.0f;
    static constexpr float kConfirmDistanceMm = 10.0f;
    static constexpr int kSampleCount = 7;
    static constexpr float kWindowMm = (kSampleCount - 1) * kSampleStepMm;

    void reset()
    {
        count_ = 0;
        newest_ = 0;
        phase_mm_ = 0.0f;
        previous_distance_mm_ = 0.0f;
        difference_mm_ = 0.0f;
        stable_distance_mm_ = 0.0f;
    }

    void update(float distance_mm, float forward_step_mm, bool valid)
    {
        if (!valid || !isfinite(distance_mm) || !isfinite(forward_step_mm)
            || forward_step_mm < 0.0f)
        {
            reset();
            return;
        }

        // A gap larger than the window cannot establish spatial stability.
        if (count_ == 0 || forward_step_mm > kWindowMm)
        {
            reset();
            samples_[0] = distance_mm;
            previous_distance_mm_ = distance_mm;
            count_ = 1;
            return;
        }

        const float missing_mm = count_ < kSampleCount
            ? (kSampleCount - count_) * kSampleStepMm - phase_mm_ : 0.0f;
        if (forward_step_mm > 0.0f)
        {
            float boundary_mm = kSampleStepMm - phase_mm_;
            int crossed = 0;
            while (boundary_mm <= forward_step_mm + kSampleEpsilonMm)
            {
                const float ratio = boundary_mm < forward_step_mm
                    ? boundary_mm / forward_step_mm : 1.0f;
                newest_ = (newest_ + 1) % kSampleCount;
                samples_[newest_] = previous_distance_mm_
                    + ratio * (distance_mm - previous_distance_mm_);
                if (count_ < kSampleCount) ++count_;
                ++crossed;
                boundary_mm += kSampleStepMm;
            }
            phase_mm_ += forward_step_mm - crossed * kSampleStepMm;
            if (phase_mm_ < 0.0f) phase_mm_ = 0.0f;
        }
        previous_distance_mm_ = distance_mm;

        if (count_ < kSampleCount) return;

        float sum = distance_mm;
        const float ratio = phase_mm_ / kSampleStepMm;
        for (int offset = 1; offset < kSampleCount; ++offset)
        {
            const int older = (newest_ + kSampleCount - offset) % kSampleCount;
            const int newer = (older + 1) % kSampleCount;
            sum += samples_[older] + ratio * (samples_[newer] - samples_[older]);
        }
        difference_mm_ = distance_mm - sum / kSampleCount;
        if (fabsf(difference_mm_) < kDifferenceLimitMm)
        {
            // Do not count the part of this step that only filled the window.
            const float confirmed_mm = forward_step_mm > missing_mm
                ? forward_step_mm - missing_mm : 0.0f;
            stable_distance_mm_ += confirmed_mm;
            if (stable_distance_mm_ >= kConfirmDistanceMm - kDistanceEpsilonMm)
                stable_distance_mm_ = kConfirmDistanceMm;
        }
        else
        {
            stable_distance_mm_ = 0.0f;
        }
    }

    bool ready() const { return stable_distance_mm_ >= kConfirmDistanceMm; }
    float difference_mm() const { return difference_mm_; }
    float stable_distance_mm() const { return stable_distance_mm_; }

private:
    // Float roundoff must not add a whole sensor cycle at an exact mm boundary.
    static constexpr float kSampleEpsilonMm = 0.000001f;
    static constexpr float kDistanceEpsilonMm = 0.0001f;
    float samples_[kSampleCount] = {};
    int count_ = 0;
    int newest_ = 0;
    float phase_mm_ = 0.0f;
    float previous_distance_mm_ = 0.0f;
    float difference_mm_ = 0.0f;
    float stable_distance_mm_ = 0.0f;
};

// 直進で共通に使うOpen起点の履歴。短い近接を連続壁と混同しない。
// Open起点の接近を、幅だけで連続壁へ格上げしない。
// 長い壁の再取得には安定した測距を要求し、保留は1区画までに制限する。
class StraightFeatureHistory {
public:
    void reset() { *this = StraightFeatureHistory{}; }
    bool open_origin() const { return recent_open_; }
    bool update(float range_mm, float step_mm, bool valid, bool motion_valid = true, bool metric_valid = true) {
        const auto &p = straight_feature_history_param;
        if (!valid || !isfinite(range_mm) || range_mm <= 0 || !isfinite(step_mm)
            || step_mm < 0 || step_mm > wall_edge_trial_param.maximum_sample_step_mm) {
            reset(); return false;
        }
        const bool open = range_mm >= p.open_mm;
        if (!initialized_) { initialized_ = true; previous_open_ = open; return false; }
        if (step_mm == 0) {
            if (++idle_ticks_ > wall_edge_trial_param.recent_forward_ticks) reset();
            else if (!open) { open_travel_mm_ = 0; previous_open_ = false; }
            return recent_open_;
        }
        idle_ticks_ = 0;
        // Geometry history survives yaw motion; the peak detector separately
        // taints the observation quality while rotating.
        open_travel_mm_ = open && previous_open_
            ? fminf(p.open_travel_mm, open_travel_mm_ + step_mm) : 0;
        if (!metric_valid || !motion_valid || range_mm > wall_edge_trial_param.arm_range_mm) {
            wall_anchor_ = false; wall_stable_mm_ = wall_window_mm_ = 0;
        } else if (!wall_anchor_) {
            wall_anchor_ = true; wall_anchor_mm_ = range_mm; wall_window_mm_ = 0;
        } else {
            wall_window_mm_ += step_mm;
            if (wall_window_mm_ >= wall_control_param.derivative_window_m * 1000.0f) {
                const float slope = fabsf(range_mm-wall_anchor_mm_) / wall_window_mm_;
                wall_stable_mm_ = slope <= wall_control_param.residual_accept
                    ? fminf(wall_edge_trial_param.arm_travel_mm, wall_stable_mm_+wall_window_mm_) : 0;
                wall_anchor_mm_ = range_mm; wall_window_mm_ = 0;
            }
        }
        if (open_travel_mm_ >= p.open_travel_mm) {
            recent_open_ = true; excursion_mm_ = 0;
        } else if (recent_open_) {
            excursion_mm_ += step_mm;
            if ((excursion_mm_ > p.maximum_excursion_mm && wall_stable_mm_ >= wall_edge_trial_param.arm_travel_mm)
                || excursion_mm_ > static_cast<float>(SECTION)) recent_open_ = false;
        }
        previous_open_ = open;
        return recent_open_;
    }
private:
    float open_travel_mm_ = 0, excursion_mm_ = 0;
    float wall_anchor_mm_ = 0, wall_window_mm_ = 0, wall_stable_mm_ = 0;
    uint16_t idle_ticks_ = 0;
    bool initialized_ = false, previous_open_ = false, recent_open_ = false, wall_anchor_ = false;
};

// 直進 Wall->Open
// One event per confirmed wall. The event position is the interpolated 65 mm
// crossing; delay_mm is forward encoder travel since that crossing.
class StraightWallEdge
{
public:
    struct Event { bool detected = false; float delay_mm = 0.0f; uint16_t age_ticks = 0; };

    void reset() { *this = StraightWallEdge{}; }
    bool pending() const { return pending_; }
    void discard_pending() {
        if (pending_) { pending_ = armed_ = false; wall_travel_mm_ = 0.0f; }
    }

    Event update(float range_mm, float forward_step_mm, bool valid = true,
                 const t_wall_edge_param &parameter = wall_edge_param,
                 bool classify_open_history = false, bool metric_valid = true, bool motion_valid = true)
    {
        Event result;
        if (!valid || !isfinite(range_mm) || !isfinite(forward_step_mm)
            || range_mm <= 0.0f || forward_step_mm < 0.0f
            || forward_step_mm > parameter.maximum_sample_step_mm) {
            reset();
            return result;
        }
        const float previous = previous_range_mm_;
        previous_range_mm_ = range_mm;
        const bool open_excursion = classify_open_history
            && feature_history_.update(range_mm, forward_step_mm, valid, motion_valid, metric_valid);
        if (!motion_valid) {
            // Keep the spatial Open identity, but never arm/confirm an edge in yaw motion.
            pending_ = armed_ = previous_valid_ = false; wall_travel_mm_ = 0; return result;
        }
        if (!previous_valid_) { previous_valid_ = true; return result; }
        if (open_excursion && range_mm > parameter.crossing_mm) {
            // Retain wall evidence below the crossing in case the excursion
            // becomes a long wall, but never publish a short Open-origin edge.
            pending_ = armed_ = false; wall_travel_mm_ = 0;
            return result;
        }
        if (forward_step_mm == 0.0f) {
            if (idle_ticks_ < 65535U) ++idle_ticks_;
            if (classify_open_history && idle_ticks_ > parameter.recent_forward_ticks) {
                pending_ = armed_ = false; wall_travel_mm_ = 0;
                return result;
            }
            // A zero encoder increment is common between pulses at low speed.
            // It neither confirms an edge nor discards an otherwise valid one.
            if (pending_) {
                pending_age_ticks_ += 1.0f;
                if (range_mm <= parameter.crossing_mm) pending_ = false;
                else if (pending_age_ticks_ >= parameter.maximum_pending_ticks) {
                    pending_ = armed_ = false; wall_travel_mm_ = 0.0f;
                }
            } else if (armed_ && idle_ticks_ <= parameter.recent_forward_ticks
                       && previous <= parameter.crossing_mm && range_mm > parameter.crossing_mm) {
                // Preserve an IR crossing between encoder pulses. No sub-count
                // position can be inferred; later forward travel must confirm it.
                pending_travel_mm_ = pending_age_ticks_ = 0.0f;
                pending_ = true;
            } else if (range_mm > parameter.crossing_mm) { armed_ = false; wall_travel_mm_ = 0.0f; }
            return result;
        }
        idle_ticks_ = 0;
        if (!armed_) {
            wall_travel_mm_ = range_mm <= parameter.arm_range_mm && previous <= parameter.arm_range_mm
                ? fminf(parameter.arm_travel_mm, wall_travel_mm_ + forward_step_mm) : 0.0f;
            armed_ = wall_travel_mm_ >= parameter.arm_travel_mm;
        }
        if (pending_) {
            if (range_mm <= parameter.crossing_mm) {
                pending_ = false; // A threshold rebound / single-sample spike.
            } else {
                pending_travel_mm_ += forward_step_mm;
                pending_age_ticks_ += 1.0f;
                if (pending_travel_mm_ > parameter.maximum_pending_mm || pending_age_ticks_ >= parameter.maximum_pending_ticks) {
                    pending_ = armed_ = false;
                    wall_travel_mm_ = 0.0f;
                } else if (range_mm >= parameter.confirm_range_mm && pending_travel_mm_ >= parameter.confirm_travel_mm) {
                    result.detected = true;
                    result.delay_mm = pending_travel_mm_;
                    result.age_ticks = static_cast<uint16_t>(fminf(65535.0f, pending_age_ticks_ + 0.5f));
                    pending_ = armed_ = false;
                    wall_travel_mm_ = 0.0f;
                }
            }
        } else if (armed_ && previous <= parameter.crossing_mm && range_mm > parameter.crossing_mm) {
            // Linear interpolation uses raw ranges, so there is no ADC-average
            // group delay. Confirmation always requires a later moving sample.
            const float fraction = (range_mm - parameter.crossing_mm) / (range_mm - previous);
            pending_travel_mm_ = forward_step_mm * fraction;
            pending_age_ticks_ = fraction;
            pending_ = true;
        }
        return result;
    }

private:
    StraightFeatureHistory feature_history_;
    float previous_range_mm_ = 0.0f, wall_travel_mm_ = 0.0f;
    float pending_travel_mm_ = 0.0f, pending_age_ticks_ = 0.0f;
    uint16_t idle_ticks_ = 65535U;
    bool previous_valid_ = false, armed_ = false, pending_ = false;
};

// 斜めの空間差分交差（既存方式）
// A diagonal pillar produces a range minimum, not a straight-wall 65mm edge.
// Use a fixed spatial window; report the zero crossing and travel since it.
class DiagonalWallEdge {
public:
    struct Event { bool detected = false; float delay_mm = 0.0f; uint16_t age_ticks = 0; };
    static constexpr int sample_count = 11;
    void reset() { *this = DiagonalWallEdge{}; }
    void discard_pending() {
        if (pending_) { pending_ = armed_ = false; arm_travel_ = 0.0f; }
    }
    Event update(float range_mm, float travel_mm, bool valid = true) {
        Event out;
        const auto &p = diagonal_edge_param;
        if (!valid || !isfinite(range_mm) || range_mm <= 0.0f || !isfinite(travel_mm)
            || travel_mm < 0.0f || travel_mm > p.maximum_sample_step_mm
            || p.sample_step_mm < .5f || p.sample_step_mm > p.maximum_sample_step_mm) {
            reset(); return out;
        }
        if (!count_) {
            samples_[0] = previous_range_ = range_mm; count_ = 1; return out;
        }
        if (travel_mm == 0.0f) {
            if (++idle_ticks_ > p.recent_forward_ticks) { reset(); return out; }
            if (pending_) {
                pending_age_ += 1.0f;
                if (range_mm < crossing_range_) pending_ = false;
            }
            previous_range_ = range_mm;
            return out; // Quantized zero counts never arm or confirm a corner.
        }
        idle_ticks_ = 0;
        const float previous_range = previous_range_;
        float boundary = p.sample_step_mm - phase_;
        int crossed = 0;
        while (boundary <= travel_mm + .000001f) {
            const float fraction = fminf(1.0f, boundary / travel_mm);
            newest_ = (newest_ + 1) % sample_count;
            samples_[newest_] = previous_range + fraction * (range_mm - previous_range);
            if (count_ < sample_count) ++count_;
            ++crossed; boundary += p.sample_step_mm;
        }
        phase_ = fmaxf(0.0f, phase_ + travel_mm - crossed * p.sample_step_mm);
        previous_range_ = range_mm;
        if (count_ < sample_count) return out;
        float sum = range_mm;
        const float fraction = phase_ / p.sample_step_mm;
        for (int offset = 1; offset < sample_count; ++offset) {
            const int older = (newest_ + sample_count - offset) % sample_count;
            const int newer = (older + 1) % sample_count;
            sum += samples_[older] + fraction * (samples_[newer] - samples_[older]);
        }
        const float difference = range_mm - sum / sample_count;
        const float previous_difference = difference_;
        difference_ = difference;
        if (!difference_valid_) { difference_valid_ = true; return out; }
        if (latched_) {
            since_event_travel_ = fminf(p.rearm_travel_mm, since_event_travel_ + travel_mm);
            release_travel_ = range_mm >= p.release_mm && previous_range >= p.release_mm
                ? release_travel_ + travel_mm : 0.0f;
            const bool separated = since_event_travel_ >= p.rearm_travel_mm
                && range_mm >= crossing_range_ + p.release_rise_mm;
            if (release_travel_ >= p.release_travel_mm || separated) { latched_ = false; release_travel_ = 0.0f; }
            return out;
        }
        if (!armed_) {
            arm_travel_ = range_mm < p.near_mm && difference <= -p.arm_difference_mm
                && previous_difference <= -p.arm_difference_mm ? arm_travel_ + travel_mm : 0.0f;
            armed_ = arm_travel_ >= p.arm_travel_mm;
        }
        if (pending_) {
            pending_travel_ += travel_mm;
            pending_age_ += 1.0f;
            if (pending_travel_ > p.maximum_pending_mm || pending_age_ >= p.maximum_pending_ticks) {
                pending_ = armed_ = false; arm_travel_ = 0.0f;
            } else if (difference < 0.0f || range_mm < crossing_range_) {
                pending_ = false; // A rebound/spike must not become a corner.
            } else if (range_mm >= previous_range - p.rebound_mm
                && difference >= p.confirm_difference_mm && range_mm - crossing_range_ >= p.confirm_rise_mm
                && pending_travel_ >= p.confirm_travel_mm) {
                out.detected = true; out.delay_mm = pending_travel_;
                out.age_ticks = static_cast<uint16_t>(fminf(65535.0f, pending_age_ + .5f));
                latched_ = true; pending_ = armed_ = false; arm_travel_ = release_travel_ = 0.0f;
                since_event_travel_ = 0.0f;
            }
        } else if (armed_ && previous_difference <= 0.0f && difference > 0.0f) {
            const float after_crossing = difference / (difference - previous_difference);
            crossing_range_ = range_mm - after_crossing * (range_mm - previous_range);
            if (crossing_range_ < p.near_mm) {
                pending_travel_ = travel_mm * after_crossing;
                pending_age_ = after_crossing; pending_ = true;
            }
        }
        return out;
    }
private:
    float samples_[sample_count] = {};
    float phase_ = 0.0f, previous_range_ = 0.0f, difference_ = 0.0f;
    float arm_travel_ = 0.0f, release_travel_ = 0.0f;
    float since_event_travel_ = 0.0f;
    float crossing_range_ = 0.0f, pending_travel_ = 0.0f, pending_age_ = 0.0f;
    int count_ = 0, newest_ = 0;
    uint16_t idle_ticks_ = 0;
    bool difference_valid_ = false, armed_ = false, pending_ = false, latched_ = false;
};

// 柱の極値・平坦区間の共通処理
// O(1), no allocation or initial 10mm buffer fill. A peak represents an
// observed point/flat-top interval; confirmation has a separate coordinate.
class SpatialPillarPeak {
public:
    struct Event {
        bool detected = false, peak_updated = false, motion_clean = false;
        uint16_t context_flags = 0;
        float delay_mm = 0, sample_delay_mm = 0, width_mm = 0;
        float peak_signal = 0, prominence = 0;
    };
    void reset() { *this = SpatialPillarPeak{}; }
    bool pending() const { return active_; }
    Event update(float range_mm, int16_t adc, float travel_mm, bool sample_valid,
                 bool motion_valid, const PillarPeakParam &p, uint16_t context_flags = 0, bool qualified_open = false) {
        Event out;
        if (!sample_valid || adc < 0 || !isfinite(range_mm) || range_mm <= 0
            || !isfinite(travel_mm) || travel_mm < 0 || travel_mm > 6.0f) {
            reset(); return out;
        }
        const float value = p.adc_peak ? static_cast<float>(adc) : -range_mm;
        if (!initialized_) {
            initialized_ = true; previous_ = baseline_ = value;
            previous_open_ = range_mm >= p.open_mm; return out;
        }
        if (travel_mm == 0) {
            if (idle_ < 5U) ++idle_;
            if (idle_ > 4U) {
                active_ = open_ready_ = false; open_travel_ = rise_travel_ = 0;
                baseline_ = value;
                approach_clean_ = motion_valid;
                approach_flags_ = 0;
            }
            // No zero-count sample can arm, move an extremum, or confirm it.
            if (active_ && !motion_valid) motion_clean_ = false;
            if (active_) active_flags_ |= context_flags;
            return out;
        }
        idle_ = 0; position_ += travel_mm;
        const bool open = range_mm >= p.open_mm;
        if (latched_) {
            since_event_ += travel_mm;
            // Both distance separation AND an actual departure are required.
            if (since_event_ >= p.separation_mm && peak_ - value >= p.release_signal
                && (!p.require_open || open)) {
                latched_ = false; active_ = open_ready_ = false;
                open_travel_ = rise_travel_ = 0; baseline_ = value;
                approach_clean_ = motion_valid;
                approach_flags_ = 0;
            }
            previous_ = value; previous_open_ = open; return out;
        }
        if (!active_) {
            if (p.require_open) {
                open_travel_ = open && previous_open_ && motion_valid ? open_travel_ + travel_mm : 0;
                if (open_travel_ >= p.open_travel_mm) open_ready_ = true;
                if (!motion_valid) open_ready_ = false;
                if (qualified_open) open_ready_ = true;
            }
            if (value < baseline_) baseline_ = value;
            if (value > previous_ + p.signal_epsilon) rise_travel_ += travel_mm;
            else if (value < previous_ - p.top_tolerance) { rise_travel_ = 0; approach_clean_ = motion_valid; approach_flags_ = 0; }
            if (!motion_valid) approach_clean_ = false;
            if (rise_travel_ > 0) approach_flags_ |= context_flags;
            if ((!p.require_open || open_ready_) && range_mm <= p.near_mm
                && value - baseline_ >= p.approach_signal && rise_travel_ >= p.approach_travel_mm) {
                active_ = true; motion_clean_ = approach_clean_ && motion_valid; departing_ = false;
                active_flags_ = approach_flags_ | context_flags;
                peak_position_ = top_start_ = top_end_ = position_;
                peak_ = top_start_level_ = value; departure_ = 0; out.peak_updated = true;
            }
        } else {
            if (!motion_valid) motion_clean_ = false;
            active_flags_ |= context_flags;
            if (position_ - peak_position_ > p.maximum_after_peak_mm) {
                active_ = open_ready_ = false; rise_travel_ = open_travel_ = 0; baseline_ = value;
                approach_clean_ = motion_valid;
                approach_flags_ = 0;
            } else {
                if (value > peak_) {
                    peak_ = value; peak_position_ = position_; out.peak_updated = true;
                    if (peak_ - top_start_level_ > p.top_tolerance) {
                        top_start_ = position_; top_start_level_ = value;
                    }
                }
                if (value >= peak_ - p.top_tolerance) {
                    top_end_ = position_; departing_ = false; departure_ = 0;
                } else if (!departing_) { departing_ = true; departure_ = 0; }
                else departure_ += travel_mm;
                const bool returned = !p.require_open || (open && value <= baseline_ + p.return_fraction * (peak_ - baseline_));
                if (departing_ && returned && peak_ - value >= p.confirm_signal
                    && departure_ >= p.confirm_travel_mm) {
                    out.detected = true; out.motion_clean = motion_clean_;
                    out.context_flags = active_flags_;
                    out.delay_mm = position_ - .5f * (top_start_ + top_end_);
                    out.sample_delay_mm = position_ - peak_position_;
                    out.width_mm = top_end_ - top_start_;
                    out.peak_signal = peak_; out.prominence = peak_ - baseline_;
                    latched_ = true; active_ = false; since_event_ = 0;
                }
            }
        }
        previous_ = value; previous_open_ = open; return out;
    }
private:
    bool initialized_ = false, active_ = false, latched_ = false, open_ready_ = false;
    bool motion_clean_ = true, approach_clean_ = true, previous_open_ = false, departing_ = false;
    uint16_t idle_ = 0;
    uint16_t approach_flags_ = 0, active_flags_ = 0;
    float position_ = 0, previous_ = 0, baseline_ = 0, peak_ = 0, top_start_level_ = 0;
    float open_travel_ = 0, rise_travel_ = 0;
    float peak_position_ = 0, top_start_ = 0, top_end_ = 0, departure_ = 0, since_event_ = 0;
};

// 直進 Open->Open
// Open->Open: qualify open travel BEFORE the ADC peak and verify open AFTER.
// Uses ADC so weak pillars beyond the calibrated 80mm range remain observable.
class StraightPillarEdge {
    SpatialPillarPeak peak_;
    StraightFeatureHistory feature_history_;
public:
    using Event = SpatialPillarPeak::Event;
    void reset() { peak_.reset(); feature_history_.reset(); }
    bool pending() const { return peak_.pending(); }
    bool open_origin() const { return feature_history_.open_origin(); }
    Event update(float range_mm, int16_t adc, float travel_mm, bool sample_valid = true, bool motion_valid = true, uint16_t context_flags = 0, bool metric_valid = true) {
        const bool recent_open = feature_history_.update(range_mm,travel_mm,sample_valid && adc >= 0,motion_valid,metric_valid);
        if (!recent_open && range_mm < straight_feature_history_param.open_mm) {
            // Expire stale Open on a long wall, but preserve a close, short
            // pillar even when it remains below 62mm for more than 12mm.
            peak_.reset(); return Event{};
        }
        return peak_.update(range_mm, adc, travel_mm, sample_valid, motion_valid, straight_pillar_param, context_flags,recent_open);
    }
};

// 斜めの距離極小
// Separate feature from DiagonalWallEdge's spatial-difference zero crossing.
// A bottom interval survives a plateau; rotating samples taint its quality.
class DiagonalPillarEdge {
    SpatialPillarPeak peak_;
public:
    bool pending() const { return peak_.pending(); }
    using Event = SpatialPillarPeak::Event;
    void reset() { peak_.reset(); }
    Event update(float range_mm, int16_t adc, float travel_mm, bool sample_valid = true, bool motion_valid = true, uint16_t context_flags = 0) {
        return peak_.update(range_mm, adc, travel_mm, sample_valid, motion_valid, diagonal_pillar_param, context_flags);
    }
};

#endif
