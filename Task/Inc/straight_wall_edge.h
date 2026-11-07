#ifndef STRAIGHT_WALL_EDGE_H_
#define STRAIGHT_WALL_EDGE_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

// One event per confirmed wall. The event position is the interpolated 65 mm
// crossing; delay_mm is forward encoder travel since that crossing.
class StraightWallEdge
{
public:
    struct Event { bool detected = false; float delay_mm = 0.0f; uint16_t age_ticks = 0; };

    void reset() { *this = StraightWallEdge{}; }
    bool pending() const { return pending_; }

    Event update(float range_mm, float forward_step_mm, bool valid = true)
    {
        Event result;
        if (!valid || !isfinite(range_mm) || !isfinite(forward_step_mm)
            || range_mm <= 0.0f || forward_step_mm < 0.0f
            || forward_step_mm > wall_edge_param.maximum_sample_step_mm) {
            reset();
            return result;
        }
        const float previous = previous_range_mm_;
        previous_range_mm_ = range_mm;
        if (!previous_valid_) { previous_valid_ = true; return result; }
        if (forward_step_mm == 0.0f) {
            if (idle_ticks_ < 65535U) ++idle_ticks_;
            // A zero encoder increment is common between pulses at low speed.
            // It neither confirms an edge nor discards an otherwise valid one.
            if (pending_) {
                pending_age_ticks_ += 1.0f;
                if (range_mm <= wall_edge_param.crossing_mm) pending_ = false;
                else if (pending_age_ticks_ >= wall_edge_param.maximum_pending_ticks) {
                    pending_ = armed_ = false; wall_travel_mm_ = 0.0f;
                }
            } else if (armed_ && idle_ticks_ <= wall_edge_param.recent_forward_ticks
                       && previous <= wall_edge_param.crossing_mm && range_mm > wall_edge_param.crossing_mm) {
                // Preserve an IR crossing between encoder pulses. No sub-count
                // position can be inferred; later forward travel must confirm it.
                pending_travel_mm_ = pending_age_ticks_ = 0.0f;
                pending_ = true;
            } else if (range_mm > wall_edge_param.crossing_mm) { armed_ = false; wall_travel_mm_ = 0.0f; }
            return result;
        }
        idle_ticks_ = 0;
        if (!armed_) {
            wall_travel_mm_ = range_mm <= wall_edge_param.arm_range_mm && previous <= wall_edge_param.arm_range_mm
                ? fminf(wall_edge_param.arm_travel_mm, wall_travel_mm_ + forward_step_mm) : 0.0f;
            armed_ = wall_travel_mm_ >= wall_edge_param.arm_travel_mm;
        }
        if (pending_) {
            if (range_mm <= wall_edge_param.crossing_mm) {
                pending_ = false; // A threshold rebound / single-sample spike.
            } else {
                pending_travel_mm_ += forward_step_mm;
                pending_age_ticks_ += 1.0f;
                if (pending_travel_mm_ > wall_edge_param.maximum_pending_mm || pending_age_ticks_ >= wall_edge_param.maximum_pending_ticks) {
                    pending_ = armed_ = false;
                    wall_travel_mm_ = 0.0f;
                } else if (range_mm >= wall_edge_param.confirm_range_mm && pending_travel_mm_ >= wall_edge_param.confirm_travel_mm) {
                    result.detected = true;
                    result.delay_mm = pending_travel_mm_;
                    result.age_ticks = static_cast<uint16_t>(fminf(65535.0f, pending_age_ticks_ + 0.5f));
                    pending_ = armed_ = false;
                    wall_travel_mm_ = 0.0f;
                }
            }
        } else if (armed_ && previous <= wall_edge_param.crossing_mm && range_mm > wall_edge_param.crossing_mm) {
            // Linear interpolation uses raw ranges, so there is no ADC-average
            // group delay. Confirmation always requires a later moving sample.
            const float fraction = (range_mm - wall_edge_param.crossing_mm) / (range_mm - previous);
            pending_travel_mm_ = forward_step_mm * fraction;
            pending_age_ticks_ = fraction;
            pending_ = true;
        }
        return result;
    }

private:
    float previous_range_mm_ = 0.0f, wall_travel_mm_ = 0.0f;
    float pending_travel_mm_ = 0.0f, pending_age_ticks_ = 0.0f;
    uint16_t idle_ticks_ = 65535U;
    bool previous_valid_ = false, armed_ = false, pending_ = false;
};

#endif
