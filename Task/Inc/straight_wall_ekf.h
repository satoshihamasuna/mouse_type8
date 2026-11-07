#ifndef STRAIGHT_WALL_EKF_H_
#define STRAIGHT_WALL_EKF_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

// Same passage-relative two states as the host LateralHeadingEkf: left-positive
// e [m] and psi [rad]. No global XY, velocity or bias states; fixed memory.
class StraightWallEkf {
public:
    struct Estimate {
        float lateral_m = 0.0f, heading_rad = 0.0f;
        float variance_lateral_m2 = 0.0f, covariance_m_rad = 0.0f, variance_heading_rad2 = 0.0f;
        float nis_left = 0.0f, nis_right = 0.0f, unobserved_travel_m = 0.0f;
        uint32_t wall_updates = 0, corner_updates = 0;
        uint8_t accepted_mask = 0;
        bool initialized = false, observed = false;
    };

    void clear() { estimate_ = Estimate{}; elapsed_s_ = 0.0f; }
    void start(float lateral_m, float heading_rad, float heading_variance_rad2 = -1.0f) {
        clear();
        if (!isfinite(lateral_m) || !isfinite(heading_rad)) return;
        estimate_.lateral_m = lateral_m;
        estimate_.heading_rad = heading_rad;
        estimate_.variance_lateral_m2 = wall_ekf_param.initial_lateral_std_m * wall_ekf_param.initial_lateral_std_m;
        // A new passage loses lateral position, not a valid incoming yaw prior.
        estimate_.variance_heading_rad2 = isfinite(heading_variance_rad2) && heading_variance_rad2 > 0.0f
            ? heading_variance_rad2 : wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
        estimate_.initialized = true;
    }
    bool predict(float ds_m, float delta_yaw_rad, float dt_s) {
        if (!estimate_.initialized || !isfinite(ds_m) || !isfinite(delta_yaw_rad)
            || !isfinite(dt_s) || dt_s <= 0.0f || dt_s >= 0.020f || fabsf(ds_m) > 0.020f) {
            clear(); return false;
        }
        const float mid = estimate_.heading_rad + delta_yaw_rad * 0.5f;
        const float f = ds_m * cosf(mid);
        estimate_.lateral_m += ds_m * sinf(mid);
        estimate_.heading_rad += delta_yaw_rad;
        estimate_.variance_lateral_m2 += 2.0f * f * estimate_.covariance_m_rad
            + f * f * estimate_.variance_heading_rad2 + wall_ekf_param.lateral_walk_m_sqrt_s * wall_ekf_param.lateral_walk_m_sqrt_s * dt_s;
        estimate_.covariance_m_rad += f * estimate_.variance_heading_rad2;
        estimate_.variance_heading_rad2 += wall_ekf_param.heading_walk_rad_sqrt_s * wall_ekf_param.heading_walk_rad_sqrt_s * dt_s;
        estimate_.unobserved_travel_m += fabsf(ds_m);
        elapsed_s_ += dt_s;
        return finite();
    }
    // Test both sides against the SAME prior, then assimilate their combined
    // lateral observation once. Repeated ten-point ADC averages are decimated.
    uint8_t observe_walls(float left_m, float right_m, uint8_t candidates, bool fresh) {
        if (!estimate_.initialized) return 0;
        const float left = 0.045f - left_m, right = right_m - 0.045f;
        estimate_.nis_left = nis(left, wall_ekf_param.side_variance_m2);
        estimate_.nis_right = nis(right, wall_ekf_param.side_variance_m2);
        uint8_t accepted = 0;
        if ((candidates & 1) && compatible(left, wall_ekf_param.side_variance_m2)) accepted |= 1;
        if ((candidates & 2) && compatible(right, wall_ekf_param.side_variance_m2)) accepted |= 2;
        estimate_.accepted_mask = accepted;
        if (fresh && accepted && elapsed_s_ + 1.0e-7f >= wall_ekf_param.observation_interval_s) {
            const float value = accepted == 3 ? (left + right) * 0.5f : accepted == 1 ? left : right;
            if (correct(value, accepted == 3 ? wall_ekf_param.both_variance_m2 : wall_ekf_param.side_variance_m2)) {
                ++estimate_.wall_updates; elapsed_s_ = 0.0f;
            }
        }
        return accepted;
    }
    bool observe_corner_lateral(float lateral_m) {
        if (!estimate_.initialized || !compatible(lateral_m, wall_ekf_param.corner_variance_m2)) return false;
        if (!correct(lateral_m, wall_ekf_param.corner_variance_m2)) return false;
        ++estimate_.corner_updates;
        // A corner and its smoothed side range share sensor data. Do not fuse
        // another wall observation in the same averaging interval.
        elapsed_s_ = 0.0f;
        return true;
    }
    // Diagonal pillar proximity is an approximate lateral observation, not a
    // continuous wall plane. Do not infer yaw or reduce yaw variance from it.
    uint8_t observe_proximity(float lateral_m, float variance_m2, uint8_t candidates, bool fresh) {
        estimate_.accepted_mask = 0;
        if (!estimate_.initialized || !isfinite(variance_m2) || variance_m2 <= 0.0f
            || !candidates || !compatible(lateral_m, variance_m2)) return 0;
        estimate_.accepted_mask = candidates & 3U;
        if (fresh && elapsed_s_ + 1.0e-7f >= wall_ekf_param.observation_interval_s
            && correct(lateral_m, variance_m2, false)) {
            ++estimate_.wall_updates; elapsed_s_ = 0.0f;
        }
        return estimate_.accepted_mask;
    }
    bool lateral_trusted() const {
        const float limit = wall_ekf_param.lateral_std_limit_m;
        return estimate_.observed && estimate_.initialized && estimate_.variance_lateral_m2 <= limit * limit
            && fabsf(estimate_.lateral_m) <= wall_ekf_param.lateral_mean_limit_m;
    }
    bool heading_trusted() const {
        const float limit = wall_ekf_param.heading_std_limit_rad;
        return lateral_trusted() && estimate_.variance_heading_rad2 <= limit * limit
            && fabsf(estimate_.heading_rad) <= wall_ekf_param.heading_mean_limit_rad
            && estimate_.unobserved_travel_m <= wall_ekf_param.coast_limit_m;
    }
    const Estimate &estimate() const { return estimate_; }

private:
    Estimate estimate_;
    float elapsed_s_ = 0.0f;
    bool finite() {
        if (isfinite(estimate_.lateral_m) && isfinite(estimate_.heading_rad)
            && isfinite(estimate_.variance_lateral_m2) && isfinite(estimate_.covariance_m_rad)
            && isfinite(estimate_.variance_heading_rad2)) return true;
        clear(); return false;
    }
    float nis(float value, float variance) const {
        if (!isfinite(value)) return INFINITY;
        const float residual = value - estimate_.lateral_m;
        return residual * residual / (estimate_.variance_lateral_m2 + variance);
    }
    bool compatible(float value, float variance) const {
        return isfinite(value) && fabsf(value - estimate_.lateral_m) <= wall_ekf_param.innovation_limit_m
            && nis(value, variance) <= wall_ekf_param.nis_limit;
    }
    bool correct(float value, float variance, bool infer_heading = true) {
        const float p00 = estimate_.variance_lateral_m2, p01 = estimate_.covariance_m_rad;
        const float p11 = estimate_.variance_heading_rad2;
        const float k0 = p00 / (p00 + variance), k1 = infer_heading ? p01 / (p00 + variance) : 0.0f;
        const float residual = value - estimate_.lateral_m, a = 1.0f - k0;
        estimate_.lateral_m += k0 * residual;
        estimate_.heading_rad += k1 * residual;
        // Joseph covariance update; symmetric storage of the 2x2 matrix.
        estimate_.variance_lateral_m2 = a * a * p00 + k0 * k0 * variance;
        estimate_.covariance_m_rad = a * (p01 - k1 * p00) + k0 * k1 * variance;
        estimate_.variance_heading_rad2 = p11 - 2.0f * k1 * p01 + k1 * k1 * (p00 + variance);
        estimate_.unobserved_travel_m = 0.0f;
        estimate_.observed = true;
        return finite();
    }
};
#endif
