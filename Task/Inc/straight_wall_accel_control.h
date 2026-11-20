#ifndef TASK_INC_STRAIGHT_WALL_ACCEL_CONTROL_H_
#define TASK_INC_STRAIGHT_WALL_ACCEL_CONTROL_H_

#include <math.h>
#include <stdint.h>

// 未接続の直進壁制御候補。IRの採否・EKF・Vehicle・モータI/Oから独立する。
// e, psi, omega, alphaは左正。距離[m]、姿勢[rad]、時間[s]を使用する。
// 固定L・直線通路・横滑りなしのモデル:
//   s = e + L*psi
//   s_dot = v*sin(psi) + L*omega_measured
//   s_ddot = a*sin(psi) + v*cos(psi)*omega_measured + L*alpha
// s_ddot + Kd*s_dot + Kp*s = 0となるalphaを求め、指令omegaを積分する。
// パラメータは構築時に固定する。A/Bの実機採用値・自動接続は持たない。
class StraightWallAccelControl
{
public:
    struct Parameters {
        float lookahead_m = 0.0f;
        float error_kp_per_s2 = 0.0f;
        float error_kd_per_s = 0.0f;
        float omega_limit_radps = 0.0f;
        float alpha_limit_radps2 = 0.0f;
        float minimum_velocity_mps = 0.20f;
    };
    struct Input {
        float lateral_m = 0.0f, heading_rad = 0.0f;
        float velocity_mps = 0.0f, accel_mps2 = 0.0f;
        float measured_omega_radps = 0.0f;
        float dt_s = 0.001f;
        // 呼出し側が、同じ通路を基準とした姿勢・横位置の採用可否を決める。
        bool enabled = false;
    };
    enum class Status : uint8_t { Disabled, InvalidParameters, InvalidInput, LowSpeed, Active };
    struct Output {
        float omega_radps = 0.0f, alpha_radps2 = 0.0f;
        float error_m = 0.0f, error_rate_mps = 0.0f;
        float requested_alpha_radps2 = 0.0f;
        Status status = Status::Disabled;
        bool omega_saturated = false, alpha_saturated = false;
    };

    explicit StraightWallAccelControl(const Parameters &parameters) : parameters_(parameters) {}

    // 切替時に前の指令を明示的に引き継げる。範囲外/不正な初期指令は拒否し0へ戻す。
    // 無効化・異常・低速の出力は0であり、Active中の変化量制限とは区別する。
    bool reset(float command_omega_radps = 0.0f) {
        omega_radps_ = 0.0f;
        if (!valid_parameters() || !isfinite(command_omega_radps)
            || fabsf(command_omega_radps) > parameters_.omega_limit_radps) return false;
        omega_radps_ = command_omega_radps;
        return true;
    }

    Output update(const Input &in) {
        if (!in.enabled) return stop(Status::Disabled);
        if (!valid_parameters()) return stop(Status::InvalidParameters);
        if (!isfinite(in.lateral_m) || !isfinite(in.heading_rad)
            || !isfinite(in.velocity_mps) || !isfinite(in.accel_mps2)
            || !isfinite(in.measured_omega_radps) || !isfinite(in.dt_s)
            || in.dt_s <= 0.0f || in.dt_s >= 0.020f) return stop(Status::InvalidInput);
        if (in.velocity_mps <= parameters_.minimum_velocity_mps) return stop(Status::LowSpeed);

        Output out;
        const float sin_heading = sinf(in.heading_rad), cos_heading = cosf(in.heading_rad);
        const float lateral_velocity = in.velocity_mps * sin_heading;
        out.error_m = in.lateral_m + parameters_.lookahead_m * in.heading_rad;
        out.error_rate_mps = lateral_velocity + parameters_.lookahead_m * in.measured_omega_radps;
        const float motion_accel = in.accel_mps2 * sin_heading
            + in.velocity_mps * cos_heading * in.measured_omega_radps;
        out.requested_alpha_radps2 = -(parameters_.error_kp_per_s2 * out.error_m
            + parameters_.error_kd_per_s * out.error_rate_mps + motion_accel) / parameters_.lookahead_m;
        if (!isfinite(out.error_m) || !isfinite(out.error_rate_mps)
            || !isfinite(out.requested_alpha_radps2)) return stop(Status::InvalidInput);

        const float alpha = limit(out.requested_alpha_radps2, parameters_.alpha_limit_radps2);
        const float candidate = omega_radps_ + alpha * in.dt_s;
        if (!isfinite(candidate)) return stop(Status::InvalidInput);
        out.omega_radps = limit(candidate, parameters_.omega_limit_radps);
        // 角速度上限で失われた積分分を保持せず、FFに渡すalphaも指令増分へ合わせる。
        out.alpha_radps2 = (out.omega_radps - omega_radps_) / in.dt_s;
        if (!isfinite(out.alpha_radps2)) return stop(Status::InvalidInput);
        out.alpha_saturated = alpha != out.requested_alpha_radps2;
        out.omega_saturated = out.omega_radps != candidate;
        out.status = Status::Active;
        omega_radps_ = out.omega_radps;
        return out;
    }

private:
    const Parameters parameters_;
    float omega_radps_ = 0.0f;

    static float limit(float value, float bound) { return fmaxf(-bound, fminf(bound, value)); }
    bool valid_parameters() const {
        return isfinite(parameters_.lookahead_m) && parameters_.lookahead_m > 0.0f
            && isfinite(parameters_.error_kp_per_s2) && parameters_.error_kp_per_s2 > 0.0f
            && isfinite(parameters_.error_kd_per_s) && parameters_.error_kd_per_s > 0.0f
            && isfinite(parameters_.omega_limit_radps) && parameters_.omega_limit_radps > 0.0f
            && isfinite(parameters_.alpha_limit_radps2) && parameters_.alpha_limit_radps2 > 0.0f
            && isfinite(parameters_.minimum_velocity_mps) && parameters_.minimum_velocity_mps >= 0.0f;
    }
    Output stop(Status status) {
        omega_radps_ = 0.0f;
        Output out;
        out.status = status;
        return out;
    }
};

#endif
