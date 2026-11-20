#ifndef TASK_INC_STRAIGHT_WALL_ACCEL_ADAPTER_H_
#define TASK_INC_STRAIGHT_WALL_ACCEL_ADAPTER_H_

#include "Task/Inc/straight_wall_control.h"
#include "Task/Inc/straight_wall_accel_control.h"

// 未接続の差替え用アダプタ。Input/Output/Phaseとreset/prime_tracking/updateを共通化。
// 左右の採用・phase・推定位置同期は既存実装を使い、通常追従の操舵を差し替える。
class StraightWallAccelAdapter
{
public:
    using Input = StraightWallControl::Input;
    using Output = StraightWallControl::Output;
    using Phase = StraightWallControl::Phase;
    using Status = StraightWallAccelControl::Status;
    static constexpr Phase Disabled = StraightWallControl::Disabled;
    static constexpr Phase Estimated = StraightWallControl::Estimated;
    static constexpr Phase Confirming = StraightWallControl::Confirming;
    static constexpr Phase AwayOnly = StraightWallControl::AwayOnly;
    static constexpr Phase Tracking = StraightWallControl::Tracking;
    struct Parameters {
        float lookahead_m = 0.0f;
        float error_kp_per_s2 = 0.0f;
        float error_kd_per_s = 0.0f;
    };

    // 既定構築は呼出し互換のために用意する。未設定では出力を有効化しない。
    StraightWallAccelAdapter() : StraightWallAccelAdapter(Parameters{}) {}
    explicit StraightWallAccelAdapter(const Parameters &parameters)
        : accel_control_(model_parameters(parameters)), configured_(accel_control_.reset()) {}

    void reset() {
        wall_control_.reset();
        accel_control_.reset();
        acceleration_output_ = StraightWallAccelControl::Output{};
        status_ = Status::Disabled;
        using_acceleration_control_ = false;
    }
    void prime_tracking(float left_m, float right_m) {
        reset();
        wall_control_.prime_tracking(left_m, right_m);
    }
    Status status() const { return status_; }
    bool using_acceleration_control() const { return using_acceleration_control_; }
    const StraightWallAccelControl::Output &acceleration_output() const { return acceleration_output_; }

    Output update(const Input &in) {
        if (!in.enabled) return stop(Status::Disabled);
        if (!configured_) return stop(Status::InvalidParameters);
        // 追加入力は既定NaN。指令角速度を実測値の代用にせず、供給漏れを検出する。
        if (!isfinite(in.accel_mps2) || !isfinite(in.measured_omega_radps)
            || !isfinite(in.velocity_mps) || !isfinite(in.dt_s)
            || in.dt_s <= 0.0f || in.dt_s >= 0.020f) return stop(Status::InvalidInput);
        if (in.velocity_mps <= 0.20f) return stop(Status::LowSpeed);

        const float previous_omega = wall_control_.omega_;
        Output out = wall_control_.update(in);
        if (out.phase == Disabled || !isfinite(out.error_m) || !isfinite(out.raw_error_m)
            || !isfinite(out.omega_radps) || !isfinite(out.alpha_radps2)) return stop(Status::InvalidInput);
        status_ = Status::Active;
        using_acceleration_control_ = false;
        acceleration_output_ = StraightWallAccelControl::Output{};
        if (out.pose_control) {
            // 通常追従では壁喪失時の指令リセット分岐を通らないため、開始指令は同じ。
            if (!accel_control_.reset(previous_omega)) return stop(Status::InvalidInput);
            StraightWallAccelControl::Input model;
            model.enabled = true;
            model.lateral_m = -out.error_m; // 従来制御の右正[m]からモデルの左正[m]へ。
            model.heading_rad = in.heading_rad;
            model.velocity_mps = in.velocity_mps;
            model.accel_mps2 = in.accel_mps2;
            model.measured_omega_radps = in.measured_omega_radps;
            model.dt_s = in.dt_s;
            acceleration_output_ = accel_control_.update(model);
            if (acceleration_output_.status != Status::Active) return stop(acceleration_output_.status);
            const float blend = out.transition;
            // 共通の開始指令から求めた2指令を混合し、10mmの立上げと上限を維持する。
            out.omega_radps += blend * (acceleration_output_.omega_radps - out.omega_radps);
            out.alpha_radps2 = (out.omega_radps - previous_omega) / in.dt_s;
            out.alpha_saturated = (blend < 1.0f && out.alpha_saturated)
                || (blend > 0.0f && acceleration_output_.alpha_saturated);
            out.omega_saturated = (blend < 1.0f && out.omega_saturated)
                || (blend > 0.0f && acceleration_output_.omega_saturated);
            using_acceleration_control_ = blend > 0.0f;
        }
        // 確認中・AwayOnly・壁なしの各分岐にも、直前に実採用した指令を引き継ぐ。
        wall_control_.omega_ = out.omega_radps;
        return out;
    }

private:
    StraightWallControl wall_control_;
    StraightWallAccelControl accel_control_;
    const bool configured_;
    Status status_ = Status::Disabled;
    bool using_acceleration_control_ = false;
    StraightWallAccelControl::Output acceleration_output_;

    static StraightWallAccelControl::Parameters model_parameters(const Parameters &p) {
        StraightWallAccelControl::Parameters model;
        model.lookahead_m = p.lookahead_m;
        model.error_kp_per_s2 = p.error_kp_per_s2;
        model.error_kd_per_s = p.error_kd_per_s;
        model.omega_limit_radps = wall_control_param.tracking_omega_limit;
        model.alpha_limit_radps2 = wall_control_param.tracking_alpha_limit;
        return model;
    }
    Output stop(Status status) {
        reset();
        status_ = status;
        return Output{};
    }
};

#endif
