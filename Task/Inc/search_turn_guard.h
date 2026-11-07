#ifndef SEARCH_TURN_GUARD_H_
#define SEARCH_TURN_GUARD_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

class SearchTurnGuard {
public:
    enum Reason : uint8_t { Normal, Side, FrontDifference, FrontNear };
    enum Flag : uint8_t { SameSurface = 1, Confirmed = 2, ReliablePose = 4,
                         Suppressed = 8, HardDifference = 16, Legacy = 32, EkfNotUsed = 64 };
    struct History { bool same_surface = false, confirmed = false; };
    struct Input {
        float left_mm = 80.0f, right_mm = 80.0f, front_left_mm = 125.0f, front_right_mm = 125.0f;
        float lateral_mm = 0.0f, heading_rad = 0.0f, lateral_variance_mm2 = 0.0f, heading_variance_rad2 = 0.0f;
        float gyro_radps = 0.0f;
        bool right_turn = false, front_wall = false, left_wall = false, right_wall = false;
        bool ekf_control = false, pose_trusted = false, ekf_pose_used = false;
    };
    struct Decision { Input input; Reason reason = Normal; uint8_t flags = 0; };

    void clear() { *this = SearchTurnGuard{}; }
    History history() const { return {qualified_, candidate_ && candidate_travel_mm_ >= search_turn_guard_param.confirmation_mm}; }
    void update(float left_mm, float right_mm, float ds_mm, bool enabled) {
        if (!enabled || !isfinite(left_mm) || !isfinite(right_mm) || !isfinite(ds_mm)
            || ds_mm < 0.0f || ds_mm > 6.0f || left_mm < search_turn_guard_param.front_valid_min_mm || right_mm < search_turn_guard_param.front_valid_min_mm
            || left_mm > search_turn_guard_param.front_valid_max_mm || right_mm > search_turn_guard_param.front_valid_max_mm) { clear(); return; }
        still_ = ds_mm > 0.0f ? 0 : static_cast<uint8_t>(still_ + 1);
        if (still_ > 4) { clear(); return; }
        if (!anchor_) { anchor_ = true; left_anchor_mm_ = left_mm; right_anchor_mm_ = right_mm; }
        else {
            anchor_travel_mm_ += ds_mm;
            const float dl = left_mm - left_anchor_mm_, dr = right_mm - right_anchor_mm_;
            const bool consistent = fabsf(dl - dr) <= search_turn_guard_param.pair_change_difference_mm
                && fabsf(dl + anchor_travel_mm_) <= search_turn_guard_param.range_change_tolerance_mm
                && fabsf(dr + anchor_travel_mm_) <= search_turn_guard_param.range_change_tolerance_mm;
            if (!consistent || anchor_travel_mm_ >= search_turn_guard_param.confirmation_mm) {
                qualified_ = consistent && anchor_travel_mm_ >= search_turn_guard_param.confirmation_mm;
                left_anchor_mm_ = left_mm; right_anchor_mm_ = right_mm;
                anchor_travel_mm_ = 0.0f;
            }
        }
        const float difference = right_mm - left_mm;
        const int sign = difference < 0.0f ? -1 : 1;
        if (fabsf(difference) < search_turn_guard_param.front_release_mm || (candidate_ && candidate_sign_ != sign)) {
            candidate_ = false; candidate_travel_mm_ = 0.0f;
        }
        if (candidate_) candidate_travel_mm_ += ds_mm;
        else if (fabsf(difference) >= search_turn_guard_param.front_difference_mm) {
            candidate_ = true; candidate_sign_ = sign; candidate_travel_mm_ = 0.0f;
        }
    }
    static Decision decide(const Input &in, const History &history) {
        Decision result;
        result.input = in;
        if (history.same_surface) result.flags |= SameSurface;
        if (history.confirmed) result.flags |= Confirmed;
        if (!in.ekf_control) result.flags |= Legacy;
        else if (!in.ekf_pose_used) result.flags |= EkfNotUsed;
        const bool pose_reliable = in.ekf_control && in.pose_trusted
            && isfinite(in.lateral_mm) && fabsf(in.lateral_mm) <= search_turn_guard_param.centered_mm
            && isfinite(in.lateral_variance_mm2) && in.lateral_variance_mm2 >= 0.0f
            && in.lateral_variance_mm2 <= search_turn_guard_param.lateral_variance_limit_mm2
            && isfinite(in.heading_rad) && fabsf(in.heading_rad) <= search_turn_guard_param.aligned_rad
            && isfinite(in.heading_variance_rad2) && in.heading_variance_rad2 >= 0.0f
            && in.heading_variance_rad2 <= search_turn_guard_param.aligned_rad * search_turn_guard_param.aligned_rad
            && isfinite(in.gyro_radps) && fabsf(in.gyro_radps) <= search_turn_guard_param.quiet_yaw_radps;
        if (pose_reliable) result.flags |= ReliablePose;
        const float side = in.right_turn ? in.left_mm : in.right_mm;
        if ((in.right_turn ? in.left_wall : in.right_wall) && fabsf(side - 45.0f) >= search_turn_guard_param.side_error_mm) {
            result.reason = Side; return result;
        }
        if (!in.front_wall) return result;
        const float difference = fabsf(in.front_right_mm - in.front_left_mm);
        // Close approaches retain an immediate correction, including when the pair is clipped.
        if ((in.front_left_mm + in.front_right_mm) * 0.5f < search_turn_guard_param.front_near_mm) {
            result.reason = FrontNear; return result;
        }
        if (difference < search_turn_guard_param.front_difference_mm) return result;
        if (difference >= search_turn_guard_param.hard_front_difference_mm) result.flags |= HardDifference;
        // Only a small, not-yet-confirmed crossing may be bypassed. Uncertain
        // pose/history, sustained differences and legacy/shadow keep correcting.
        const bool pair_in_range = isfinite(in.front_left_mm) && isfinite(in.front_right_mm)
            && in.front_left_mm >= search_turn_guard_param.front_valid_min_mm && in.front_right_mm >= search_turn_guard_param.front_valid_min_mm
            && in.front_left_mm <= search_turn_guard_param.front_valid_max_mm && in.front_right_mm <= search_turn_guard_param.front_valid_max_mm;
        if (difference < search_turn_guard_param.hard_front_difference_mm && pair_in_range && pose_reliable && history.same_surface && !history.confirmed)
            result.flags |= Suppressed;
        else result.reason = FrontDifference;
        return result;
    }
private:
    float left_anchor_mm_ = 0.0f, right_anchor_mm_ = 0.0f, anchor_travel_mm_ = 0.0f;
    float candidate_travel_mm_ = 0.0f;
    int candidate_sign_ = 0;
    uint8_t still_ = 0;
    bool anchor_ = false, qualified_ = false, candidate_ = false;
};
#endif
