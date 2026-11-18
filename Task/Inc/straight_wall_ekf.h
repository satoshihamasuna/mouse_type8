#ifndef STRAIGHT_WALL_EKF_H_
#define STRAIGHT_WALL_EKF_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

// 通路内の左偏差e[m]、姿勢psi[rad]、encoder進行距離に対する誤差ds[m]。
// 進行座標を区間ごとにリセットしても、同じ通路のe/psiは保持する。
class StraightWallEkf {
public:
    struct Estimate {
        float lateral_m = 0.0f, heading_rad = 0.0f;
        float variance_lateral_m2 = 0.0f, covariance_m_rad = 0.0f, variance_heading_rad2 = 0.0f;
        float longitudinal_error_m = 0.0f, variance_longitudinal_m2 = 0.0f;
        float covariance_lateral_longitudinal_m2 = 0.0f, covariance_heading_longitudinal_m_rad = 0.0f;
        float longitudinal_nis = 0.0f, longitudinal_gain = 0.0f;
        float nis_left = 0.0f, nis_right = 0.0f, unobserved_travel_m = 0.0f;
        uint32_t wall_updates = 0, corner_updates = 0, longitudinal_updates = 0;
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
        reset_longitudinal();
    }
    void reset_longitudinal() {
        estimate_.longitudinal_error_m = 0.0f;
        estimate_.variance_longitudinal_m2 = wall_feature_ekf_param.initial_std_m * wall_feature_ekf_param.initial_std_m;
        estimate_.covariance_lateral_longitudinal_m2 = estimate_.covariance_heading_longitudinal_m_rad = 0.0f;
        estimate_.longitudinal_nis = estimate_.longitudinal_gain = 0.0f;
    }
    bool predict(float ds_m, float delta_yaw_rad, float dt_s) {
        if (!estimate_.initialized || !isfinite(ds_m) || !isfinite(delta_yaw_rad)
            || !isfinite(dt_s) || dt_s <= 0.0f || dt_s >= 0.020f || fabsf(ds_m) > 0.020f) {
            clear(); return false;
        }
        const float mid = estimate_.heading_rad + delta_yaw_rad * 0.5f;
        estimate_.lateral_m += ds_m * sinf(mid);
        estimate_.heading_rad += delta_yaw_rad;
        estimate_.longitudinal_error_m += ds_m * (cosf(mid) - 1.0f);
        const float f[3][3] = {{1.0f, ds_m*cosf(mid), 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, -ds_m*sinf(mid), 1.0f}};
        float p[3][3], fp[3][3] = {}, next[3][3] = {}; covariance(p);
        for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j)
            for (unsigned k=0;k<3;++k) fp[i][j] += f[i][k]*p[k][j];
        for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j)
            for (unsigned k=0;k<3;++k) next[i][j] += fp[i][k]*f[j][k];
        next[0][0] += wall_ekf_param.lateral_walk_m_sqrt_s * wall_ekf_param.lateral_walk_m_sqrt_s * dt_s;
        next[1][1] += wall_ekf_param.heading_walk_rad_sqrt_s * wall_ekf_param.heading_walk_rad_sqrt_s * dt_s;
        next[2][2] += wall_feature_ekf_param.walk_m_sqrt_m * wall_feature_ekf_param.walk_m_sqrt_m * fabsf(ds_m);
        covariance(next, true);
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
    bool observe_longitudinal(float error_m, float variance_m2) {
        estimate_.longitudinal_gain = 0.0f;
        if (!estimate_.initialized || !isfinite(error_m) || !isfinite(variance_m2) || variance_m2 <= 0.0f) return false;
        const float residual = error_m-estimate_.longitudinal_error_m;
        estimate_.longitudinal_nis = residual*residual/(estimate_.variance_longitudinal_m2+variance_m2);
        if (!isfinite(estimate_.longitudinal_nis) || estimate_.longitudinal_nis > wall_feature_ekf_param.nis_limit) return false;
        if (!correct_state(error_m, variance_m2, 2U, true, wall_feature_ekf_param.maximum_gain)) return false;
        ++estimate_.longitudinal_updates;
        return true;
    }
    // 推定した誤差をego.lengthへ注入した分だけ、誤差状態を戻す。共分散は保持。
    void inject_longitudinal(float delta_m) { estimate_.longitudinal_error_m -= delta_m; }

private:
    Estimate estimate_;
    float elapsed_s_ = 0.0f;
    bool finite() {
        if (isfinite(estimate_.lateral_m) && isfinite(estimate_.heading_rad)
            && isfinite(estimate_.variance_lateral_m2) && isfinite(estimate_.covariance_m_rad)
            && isfinite(estimate_.variance_heading_rad2) && isfinite(estimate_.longitudinal_error_m)
            && isfinite(estimate_.variance_longitudinal_m2) && isfinite(estimate_.covariance_lateral_longitudinal_m2)
            && isfinite(estimate_.covariance_heading_longitudinal_m_rad)) return true;
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
        if (!correct_state(value, variance, 0U, infer_heading)) return false;
        estimate_.unobserved_travel_m = 0.0f;
        estimate_.observed = true;
        return finite();
    }
    void covariance(float p[3][3], bool write = false) {
        if (write) {
            estimate_.variance_lateral_m2=p[0][0]; estimate_.covariance_m_rad=(p[0][1]+p[1][0])*.5f;
            estimate_.variance_heading_rad2=p[1][1]; estimate_.variance_longitudinal_m2=p[2][2];
            estimate_.covariance_lateral_longitudinal_m2=(p[0][2]+p[2][0])*.5f;
            estimate_.covariance_heading_longitudinal_m_rad=(p[1][2]+p[2][1])*.5f;
        } else {
            p[0][0]=estimate_.variance_lateral_m2; p[0][1]=p[1][0]=estimate_.covariance_m_rad;
            p[1][1]=estimate_.variance_heading_rad2; p[2][2]=estimate_.variance_longitudinal_m2;
            p[0][2]=p[2][0]=estimate_.covariance_lateral_longitudinal_m2;
            p[1][2]=p[2][1]=estimate_.covariance_heading_longitudinal_m_rad;
        }
    }
    bool correct_state(float value, float variance, unsigned axis, bool infer_heading, float maximum_gain=1.0f) {
        float p[3][3], k[3], a[3][3], ap[3][3]={}, next[3][3]={}; covariance(p);
        const float denominator=p[axis][axis]+variance;
        if (!isfinite(denominator) || denominator <= 0.0f || maximum_gain <= 0.0f || maximum_gain > 1.0f) return false;
        for (unsigned i=0;i<3;++i) k[i]=p[i][axis]/denominator;
        if (!infer_heading) k[1]=0.0f;
        const float scale=k[axis]>maximum_gain ? maximum_gain/k[axis] : 1.0f;
        for (auto &gain:k) gain*=scale;
        const float residual=value-(axis==0 ? estimate_.lateral_m : estimate_.longitudinal_error_m);
        estimate_.lateral_m+=k[0]*residual; estimate_.heading_rad+=k[1]*residual; estimate_.longitudinal_error_m+=k[2]*residual;
        if (axis==2) estimate_.longitudinal_gain=k[2];
        // Joseph form, fixed 3x3 storage. No heap allocation in the IRQ.
        for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) a[i][j]=(i==j?1.0f:0.0f)-(j==axis?k[i]:0.0f);
        for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) for (unsigned l=0;l<3;++l) ap[i][j]+=a[i][l]*p[l][j];
        for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) {
            for (unsigned l=0;l<3;++l) next[i][j]+=ap[i][l]*a[j][l];
            next[i][j]+=k[i]*k[j]*variance;
        }
        covariance(next,true);
        return finite();
    }
};
#endif
