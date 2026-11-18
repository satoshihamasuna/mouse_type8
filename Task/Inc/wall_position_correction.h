#ifndef TASK_INC_WALL_POSITION_CORRECTION_H_
#define TASK_INC_WALL_POSITION_CORRECTION_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"
#include "Task/Inc/ir_range_state.h"

// 探索ターン前の観測採否
class SearchTurnGuard {
public:
    enum Reason : uint8_t { Normal, Side, FrontDifference, FrontNear };
    enum Flag : uint8_t { SameSurface = 1, Confirmed = 2, ReliablePose = 4,
                         Suppressed = 8, HardDifference = 16, Legacy = 32, EkfNotUsed = 64 };
    // Passed checks, independent of the existing flags/reason ABI.
    enum Check : uint16_t { MetricPair=1, PairWindow=2, Centered=4, LateralCertain=8,
        Aligned=16, HeadingCertain=32, QuietYaw=64, PoseTrusted=128, EkfControl=256, PoseUsed=512 };
    struct History { bool same_surface = false, confirmed = false; float candidate_mm = 0.0f; };
    struct Input {
        float left_mm = 80.0f, right_mm = 80.0f, front_left_mm = 125.0f, front_right_mm = 125.0f;
        float lateral_mm = 0.0f, heading_rad = 0.0f, lateral_variance_mm2 = 0.0f, heading_variance_rad2 = 0.0f;
        float gyro_radps = 0.0f;
        bool right_turn = false, front_wall = false, left_wall = false, right_wall = false;
        bool ekf_control = false, pose_trusted = false, ekf_pose_used = false;
        int16_t adc[4]{};
        IrRangeState range[4] = {IrRangeState::Unknown, IrRangeState::Unknown, IrRangeState::Unknown, IrRangeState::Unknown};
    };
    struct Decision { Input input; Reason reason = Normal; uint8_t flags = 0; uint16_t checks = 0; float candidate_mm = 0.0f; };

    void clear() { *this = SearchTurnGuard{}; }
    History history() const { return {qualified_, candidate_ && candidate_travel_mm_ >= search_turn_guard_param.confirmation_mm, candidate_travel_mm_}; }
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
        result.candidate_mm = history.candidate_mm;
        if (ir_range_is_metric(in.range[2]) && ir_range_is_metric(in.range[3])) result.checks |= MetricPair;
        if (isfinite(in.front_left_mm) && isfinite(in.front_right_mm)
            && in.front_left_mm >= search_turn_guard_param.front_valid_min_mm && in.front_right_mm >= search_turn_guard_param.front_valid_min_mm
            && in.front_left_mm <= search_turn_guard_param.front_valid_max_mm && in.front_right_mm <= search_turn_guard_param.front_valid_max_mm) result.checks |= PairWindow;
        if (isfinite(in.lateral_mm) && fabsf(in.lateral_mm) <= search_turn_guard_param.centered_mm) result.checks |= Centered;
        if (isfinite(in.lateral_variance_mm2) && in.lateral_variance_mm2 >= 0 && in.lateral_variance_mm2 <= search_turn_guard_param.lateral_variance_limit_mm2) result.checks |= LateralCertain;
        if (isfinite(in.heading_rad) && fabsf(in.heading_rad) <= search_turn_guard_param.aligned_rad) result.checks |= Aligned;
        if (isfinite(in.heading_variance_rad2) && in.heading_variance_rad2 >= 0 && in.heading_variance_rad2 <= search_turn_guard_param.aligned_rad * search_turn_guard_param.aligned_rad) result.checks |= HeadingCertain;
        if (isfinite(in.gyro_radps) && fabsf(in.gyro_radps) <= search_turn_guard_param.quiet_yaw_radps) result.checks |= QuietYaw;
        if (in.pose_trusted) result.checks |= PoseTrusted;
        if (in.ekf_control) result.checks |= EkfControl;
        if (in.ekf_pose_used) result.checks |= PoseUsed;
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
        if (difference < search_turn_guard_param.hard_front_difference_mm && pair_in_range
            && (result.checks & MetricPair) && pose_reliable && history.same_surface && !history.confirmed)
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

// 壁切れの縦位置補正・Shadow採否
namespace WallPositionCorrection {
enum class Phase : uint8_t { StraightEntry, DiagonalEntry, StraightExit, DiagonalExit, SearchStraight, StraightRun, DiagonalRun };
inline bool diagonal_phase(Phase p) { return p==Phase::DiagonalEntry || p==Phase::DiagonalExit || p==Phase::DiagonalRun; }
enum class Mode : uint8_t { Legacy, Shadow, Control };
struct FeatureContext {
    bool valid = false;
    Phase phase = Phase::SearchStraight;
    float wall_reference_mm = 0.0f;
    uint8_t expected_side = 0;
    float period_mm = 0.0f;
};
inline float pillar_reference(const FeatureContext &frame) {
    // 特徴点間の差だけを移す。最短入口の原点を探索の区画原点と混同しない。
    return frame.wall_reference_mm + wall_feature_ekf_param.pillar_reference_mm - wall_edge_trial_search_reference_mm;
}
inline float feature_reference(const FeatureContext &frame, float feature_mm, bool pillar) {
    const float base = pillar ? pillar_reference(frame) : frame.wall_reference_mm;
    // 確定位置ではなく、交差/極小位置で区画を選ぶ。遅れて確定した柱を次区画へ送らない。
    return frame.period_mm > 0.0f && isfinite(feature_mm)
        ? base + fmaxf(0.0f, roundf((feature_mm-base)/frame.period_mm)) * frame.period_mm : base;
}
enum class Reason : uint8_t { Accepted, Limited, OutsideWindow, Invalid, WrongSide, Shadow, NotSelected, StaleGeneration };
inline const char *reason_name(Reason reason) {
    const char *names[] = {"accepted", "limited", "outside_window", "invalid", "wrong_side", "shadow", "not_selected", "stale_generation"};
    return names[static_cast<unsigned>(reason)];
}
inline const char *mode_name(Mode mode) { return mode == Mode::Legacy ? "legacy" : mode == Mode::Shadow ? "shadow" : "control"; }
inline const t_wall_edge_param &edge_param(Mode mode) { return mode == Mode::Legacy ? wall_edge_param : wall_edge_trial_param; }
inline float search_reference(Mode mode) { return mode == Mode::Legacy ? SEARCH_CORRECTION : wall_edge_trial_search_reference_mm; }
struct Result {
    float before_mm = 0.0f, target_mm = 0.0f, after_mm = 0.0f;
    float innovation_mm = 0.0f, delta_mm = 0.0f, delay_mm = 0.0f;
    Reason reason = Reason::Invalid;
    Reason proposed_reason = Reason::Invalid;
    float proposed_delta_mm = 0.0f, proposed_after_mm = 0.0f;
    bool ekf = false;
    float ekf_nis = 0.0f, ekf_gain = 0.0f, ekf_variance_mm2 = 0.0f;
    bool accepted() const { return reason == Reason::Accepted || reason == Reason::Limited; }
};
inline bool before_entry_origin(float feature_mm, Phase phase) {
    return phase == Phase::StraightEntry && feature_mm < 0.0f;
}
inline Result evaluate(float before_mm, float edge_reference_mm, float delay_mm, Phase phase, float feature_maximum_delay_mm = -1.0f, bool pillar = false)
{
    Result out;
    out.before_mm = out.after_mm = out.proposed_after_mm = before_mm; out.delay_mm = delay_mm;
    const unsigned index = static_cast<unsigned>(phase);
    if (index >= 7) return out;
    const auto &policy = wall_position_param[index];
    out.target_mm = edge_reference_mm + delay_mm;
    out.innovation_mm = out.target_mm - before_mm;
    const float maximum_delay = feature_maximum_delay_mm >= 0.0f ? feature_maximum_delay_mm
        : diagonal_phase(phase)
            ? diagonal_edge_param.maximum_pending_mm : wall_edge_param.maximum_pending_mm;
    if (!isfinite(before_mm) || !isfinite(out.target_mm) || !isfinite(delay_mm)
        || delay_mm < 0.0f || delay_mm > maximum_delay
        || !isfinite(policy.window_mm) || !isfinite(policy.maximum_step_mm) || !isfinite(policy.blend)
        || policy.window_mm <= 0.0f || policy.maximum_step_mm <= 0.0f || policy.blend <= 0.0f || policy.blend > 1.0f)
        return out;
    // 原点を跨いで離反確定した前直線の柱を、ターン入口の新しい観測にしない。
    // 壁切れと探索の柱は従来どおり評価する。棄却は EKF/境界ラッチ更新より前に行う。
    if (pillar && before_entry_origin(before_mm - delay_mm,phase)) return out;
    if (fabsf(out.innovation_mm) > policy.window_mm) { out.reason = Reason::OutsideWindow; return out; }
    const float desired = out.innovation_mm * policy.blend;
    out.delta_mm = fmaxf(-policy.maximum_step_mm, fminf(policy.maximum_step_mm, desired));
    out.after_mm = before_mm + out.delta_mm;
    out.reason = out.delta_mm == desired ? Reason::Accepted : Reason::Limited;
    out.proposed_reason = out.reason; out.proposed_delta_mm = out.delta_mm; out.proposed_after_mm = out.after_mm;
    return out;
}

inline void suppress(Result &result, Reason reason)
{
    result.reason = reason; result.delta_mm = 0.0f; result.after_mm = result.before_mm;
}
struct Decision {
    Result right, left;
    uint8_t selected_side = 0;
};
// Evaluate both sides against the SAME pre-correction coordinate. Rejected or
// wrong-side events must never consume the boundary before a valid event.
inline Decision decide(float before_mm, float reference_mm, float right_delay_mm, float left_delay_mm,
    uint8_t mask, uint8_t expected_side, uint32_t generation, uint32_t right_generation,
    uint32_t left_generation, Phase phase, Mode mode)
{
    Decision out;
    if (mask & 2U) {
        out.right = evaluate(before_mm, reference_mm, right_delay_mm, phase);
        if (out.right.reason != Reason::Invalid) {
            if (right_generation != generation) suppress(out.right, Reason::StaleGeneration);
            else if (expected_side && expected_side != 2U) suppress(out.right, Reason::WrongSide);
        }
    }
    if (mask & 1U) {
        out.left = evaluate(before_mm, reference_mm, left_delay_mm, phase);
        if (out.left.reason != Reason::Invalid) {
            if (left_generation != generation) suppress(out.left, Reason::StaleGeneration);
            else if (expected_side && expected_side != 1U) suppress(out.left, Reason::WrongSide);
        }
    }
    const bool right = (mask & 2U) && out.right.accepted();
    const bool left = (mask & 1U) && out.left.accepted();
    if (right && (!left || fabsf(out.right.innovation_mm) <= fabsf(out.left.innovation_mm))) out.selected_side = 2;
    else if (left) out.selected_side = 1;
    if (right && out.selected_side != 2U) suppress(out.right, Reason::NotSelected);
    if (left && out.selected_side != 1U) suppress(out.left, Reason::NotSelected);
    const bool apply = phase!=Phase::DiagonalRun && (mode == Mode::Legacy || (mode == Mode::Control
        && (phase == Phase::SearchStraight || phase == Phase::StraightEntry || phase == Phase::StraightRun)));
    if (!apply && out.selected_side) suppress(out.selected_side == 2U ? out.right : out.left, Reason::Shadow);
    return out;
}
}

// 斜めの進行距離誤差だけを並行評価する。走行用EKF/ego/制御指令への注入APIは持たない。
class DiagonalFeatureShadow {
public:
    enum Kind : uint8_t { DifferenceEdge, PillarMinimum };
    enum Status : uint8_t { NotEvaluated, Ready, Uncalibrated, QualityRejected, PoseOutside, Duplicate, WrongSide, Stale, Invalid, OutsideWindow, NisRejected };
    struct Metadata {
        bool shadow_only=false, model_ready=false, reference_valid=false;
        Status status=NotEvaluated;
        uint16_t quality_flags=0;
        float nominal_reference_mm=0, pose_shift_mm=0, left_mm=0, heading_rad=0;
    };
    struct Input {
        WallPositionCorrection::Phase phase=WallPositionCorrection::Phase::DiagonalRun;
        Kind kind=PillarMinimum;
        uint8_t side=1, expected_side=0;
        uint16_t quality_flags=0;
        uint32_t generation=0, event_generation=0;
        float before_mm=0, delay_mm=0, left_mm=0, heading_rad=0, gyro_radps=0;
    };
    struct Decision { WallPositionCorrection::Result position; Metadata model; };
    static const char *status_name(Status s) {
        const char *names[]={"not_evaluated","ready","uncalibrated","quality_rejected","pose_outside","duplicate","wrong_side","stale_generation","invalid","outside_window","nis_rejected"};
        return static_cast<unsigned>(s)<11 ? names[s] : "invalid";
    }
    static constexpr bool apply_enabled() { return false; }
    void reset() { *this=DiagonalFeatureShadow{}; }
    void begin_exit(float end_mm) {
        if (!exit_started_ && isfinite(end_mm)) { shift_[1]=end_mm; exit_started_=true; }
    }
    void set_phase(WallPositionCorrection::Phase phase) {
        if (phase==WallPositionCorrection::Phase::DiagonalEntry && phase_!=phase) shift_[0]=0;
        phase_=phase;
    }
    void rebase(float delta_mm, float reset_heading_rad=0, float reset_left_mm=0) {
        if (!isfinite(delta_mm) || !isfinite(reset_heading_rad) || !isfinite(reset_left_mm)) { reset(); return; }
        for (auto &s:shift_) s+=delta_mm;
        heading_offset_+=reset_heading_rad; left_offset_+=reset_left_mm;
    }
    float heading(float raw) const { return raw+heading_offset_; }
    float lateral(float raw) const { return raw+left_offset_; }
    void prime_heading(float heading_rad) { heading_offset_=isfinite(heading_rad)?heading_rad:0; }
    void advance(float ds_mm, float heading_rad, const DiagonalFeatureShadowParam &p) {
        if (!isfinite(ds_mm) || ds_mm<0 || ds_mm>6 || !isfinite(heading_rad)
            || !isfinite(p.initial_std_mm) || p.initial_std_mm<=0 || !isfinite(p.walk_mm_sqrt_mm) || p.walk_mm_sqrt_mm<0) { reset(); return; }
        if (!initialized_) { variance_=p.initial_std_mm*p.initial_std_mm; initialized_=true; }
        travel_+=ds_mm; mean_+=ds_mm*(cosf(heading_rad)-1.0f);
        variance_+=p.walk_mm_sqrt_mm*p.walk_mm_sqrt_mm*ds_mm;
    }
    Decision observe(const Input &in, const DiagonalFeatureShadowParam &p) {
        using namespace WallPositionCorrection;
        Decision out; auto &r=out.position; auto &m=out.model;
        r.before_mm=r.after_mm=r.proposed_after_mm=in.before_mm; r.delay_mm=in.delay_mm;
        m.shadow_only=true; m.status=Invalid; m.left_mm=in.left_mm; m.heading_rad=in.heading_rad;
        m.quality_flags=in.quality_flags;
        if (!diagonal_phase(in.phase) || in.side<1 || in.side>2 || in.kind>PillarMinimum) return out;
        const unsigned phase=in.phase==Phase::DiagonalEntry?0:in.phase==Phase::DiagonalRun?1:2;
        const auto &model=p.reference[phase][in.kind][in.side-1];
        m.model_ready=model.calibrated;
        if (!isfinite(p.maximum_lateral_mm) || p.maximum_lateral_mm<=0 || !isfinite(p.maximum_heading_rad) || p.maximum_heading_rad<=0
            || !isfinite(p.maximum_yaw_radps) || p.maximum_yaw_radps<=0 || !isfinite(p.same_pillar_mm) || p.same_pillar_mm<0) return out;
        if (in.generation!=in.event_generation) { m.status=Stale; r.reason=Reason::StaleGeneration; return out; }
        if (in.quality_flags) { m.status=QualityRejected; return out; }
        if (in.expected_side && in.expected_side!=in.side) { m.status=WrongSide; r.reason=Reason::WrongSide; return out; }
        if (!isfinite(in.before_mm) || !isfinite(in.delay_mm) || in.delay_mm<0
            || in.delay_mm>(in.kind==PillarMinimum?diagonal_pillar_param.maximum_after_peak_mm:diagonal_edge_param.maximum_pending_mm)
            || !isfinite(in.left_mm) || !isfinite(in.heading_rad) || !isfinite(in.gyro_radps)) return out;
        if (fabsf(in.left_mm)>p.maximum_lateral_mm || fabsf(in.heading_rad)>p.maximum_heading_rad || fabsf(in.gyro_radps)>p.maximum_yaw_radps) {
            m.status=PoseOutside; return out;
        }
        if (!model.calibrated) { m.status=Uncalibrated; return out; }
        if (!isfinite(model.reference_mm) || !isfinite(model.period_mm) || model.period_mm<0
            || !isfinite(model.lateral_sensitivity) || !isfinite(model.heading_sensitivity_mm)
            || !isfinite(p.window_mm) || p.window_mm<=0 || !isfinite(p.maximum_step_mm) || p.maximum_step_mm<=0
            || !isfinite(p.maximum_gain) || p.maximum_gain<=0 || p.maximum_gain>.5f || !initialized_) return out;
        m.pose_shift_mm=model.lateral_sensitivity*in.left_mm+model.heading_sensitivity_mm*in.heading_rad;
        const float base=model.reference_mm+shift_[phase];
        const float n=model.period_mm>0 ? roundf((in.before_mm-in.delay_mm-m.pose_shift_mm-base)/model.period_mm):0;
        m.nominal_reference_mm=base+n*model.period_mm;
        r.target_mm=m.nominal_reference_mm+m.pose_shift_mm+in.delay_mm;
        r.innovation_mm=r.target_mm-in.before_mm;
        if (!isfinite(r.target_mm) || !isfinite(r.innovation_mm)) return out;
        m.reference_valid=true;
        if (fabsf(r.innovation_mm)>p.window_mm) { m.status=OutsideWindow; r.reason=Reason::OutsideWindow; return out; }
        const unsigned side=in.side-1; const float path=travel_-in.delay_mm;
        if (consumed_[side] && fabsf(path-last_path_[side])<=p.same_pillar_mm) { m.status=Duplicate; r.reason=Reason::NotSelected; return out; }
        const float std_mm=in.kind==PillarMinimum?p.pillar_std_mm:p.wall_std_mm;
        if (!isfinite(std_mm) || std_mm<=0 || !isfinite(p.nis_limit) || p.nis_limit<=0 || !isfinite(variance_) || variance_<=0) return out;
        r.ekf=true; const float residual=r.innovation_mm-mean_, noise=std_mm*std_mm;
        r.ekf_nis=residual*residual/(variance_+noise);
        if (!isfinite(r.ekf_nis) || r.ekf_nis>p.nis_limit) { m.status=NisRejected; return out; }
        r.ekf_gain=fminf(p.maximum_gain,variance_/(variance_+noise));
        mean_+=r.ekf_gain*residual;
        variance_=(1-r.ekf_gain)*(1-r.ekf_gain)*variance_+r.ekf_gain*r.ekf_gain*noise;
        r.ekf_variance_mm2=variance_;
        const float limit=fminf(p.maximum_step_mm,fabsf(r.innovation_mm)*p.maximum_gain);
        r.proposed_delta_mm=fmaxf(-limit,fminf(limit,mean_)); r.proposed_after_mm=r.before_mm+r.proposed_delta_mm;
        r.proposed_reason=fabsf(mean_)>limit+1.0e-6f?Reason::Limited:Reason::Accepted;
        r.reason=Reason::Shadow; m.status=Ready;
        consumed_[side]=true; last_path_[side]=path;
        return out; // delta=0, after=before。実適用は設定値を変えても許可しない。
    }
private:
    WallPositionCorrection::Phase phase_=WallPositionCorrection::Phase::DiagonalRun;
    float shift_[3]={}, heading_offset_=0, left_offset_=0;
    float mean_=0, variance_=0, travel_=0, last_path_[2]={};
    bool initialized_=false, exit_started_=false, consumed_[2]={};
};

#endif
