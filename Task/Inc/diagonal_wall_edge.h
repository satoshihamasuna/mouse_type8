#ifndef DIAGONAL_WALL_EDGE_H_
#define DIAGONAL_WALL_EDGE_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

// A diagonal pillar produces a range minimum, not a straight-wall 65mm edge.
// Use a fixed spatial window; report the zero crossing and travel since it.
class DiagonalWallEdge {
public:
    struct Event { bool detected = false; float delay_mm = 0.0f; uint16_t age_ticks = 0; };
    static constexpr int sample_count = 11;
    void reset() { *this = DiagonalWallEdge{}; }
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

#endif
