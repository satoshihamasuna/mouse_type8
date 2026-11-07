#ifndef SEARCH_TURN_HEADING_H_
#define SEARCH_TURN_HEADING_H_

#include <math.h>
#include "Params/wall_param.h"

// New-corridor heading = incoming heading + measured yaw - logical turn angle.
// Motion owns this accumulator; IR disable and ego.radian resets cannot erase it.
class SearchTurnHeading {
    float heading_rad_ = 0.0f, logical_turn_rad_ = 0.0f;
    float variance_rad2_ = 0.0f;
    bool valid_ = false;
public:
    void clear() { heading_rad_ = logical_turn_rad_ = variance_rad2_ = 0.0f; valid_ = false; }
    void begin(float incoming_heading_rad, float logical_turn_rad, float incoming_variance_rad2 = -1.0f) {
        clear();
        if (!isfinite(incoming_heading_rad) || !isfinite(logical_turn_rad)) return;
        heading_rad_ = incoming_heading_rad;
        logical_turn_rad_ = logical_turn_rad;
        variance_rad2_ = isfinite(incoming_variance_rad2) && incoming_variance_rad2 > 0.0f
            ? incoming_variance_rad2 : wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
        valid_ = true;
    }
    void predict(float gyro_radps, float dt_s) {
        if (!valid_) return;
        if (!isfinite(gyro_radps) || !isfinite(dt_s) || dt_s <= 0.0f || dt_s >= 0.020f) {
            clear(); return;
        }
        heading_rad_ += gyro_radps * dt_s;
        variance_rad2_ += wall_ekf_param.heading_walk_rad_sqrt_s * wall_ekf_param.heading_walk_rad_sqrt_s * dt_s;
    }
    bool valid() const {
        return valid_ && isfinite(residual_rad())
            && fabsf(residual_rad()) <= search_turn_guard_param.exit_residual_limit_rad;
    }
    float residual_rad() const { return heading_rad_ - logical_turn_rad_; }
    float variance_rad2() const { return variance_rad2_; }
};
#endif
