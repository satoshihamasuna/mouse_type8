#ifndef TASK_INC_WALL_DIAGNOSTICS_H_
#define TASK_INC_WALL_DIAGNOSTICS_H_

#include <stdio.h>
#include <stdint.h>
#include "Task/Inc/wall_detection.h"
#include "Task/Inc/ir_range_state.h"
#include "Task/Inc/wall_position_correction.h"
#include "Task/Inc/diagonal_pose.h"

// 柱候補と極小時の観測記録
class PillarDiagnostics {
public:
    static constexpr unsigned capacity = 64;
    enum Flag : uint16_t { Rotating = 1, FrameChanged = 2, PositionJump = 4,
        FrontPairClose = 8, SideNearLimit = 16, InvalidFront = 32, InvalidPose = 64 };
    struct Sample {
        uint32_t ms = 0, generation = 0;
        uint32_t source_generation = 0;
        float rebase_mm = 0.0f;
        float length_mm = 0, left_mm = 0, heading_rad = 0, gyro_radps = 0;
        float control_left_mm = 0, control_heading_rad = 0;
        float model_left_mm = 0, model_heading_rad = 0;
        float distance[4] = {};
        int16_t adc[4] = {};
        IrRangeState range[4] = {};
    };
    struct Event {
        Sample minimum, confirmation;
        SpatialPillarPeak::Event peak;
        float feature_mm = 0, coordinate_jump_mm = 0;
        float reference_mm = 0, target_mm = 0, innovation_mm = 0, proposed_delta_mm = 0;
        uint16_t flags = 0;
        uint8_t axis = 0, side = 0, motion = 0, control = 0, turn = 0;
        WallPositionCorrection::Mode mode = WallPositionCorrection::Mode::Legacy;
        bool reference_valid = false, proposal_valid = false;
        bool position_evaluated = false;
        WallPositionCorrection::Phase phase = WallPositionCorrection::Phase::SearchStraight;
        WallPositionCorrection::Result position;
        DiagonalFeatureShadow::Metadata diagonal_model;
        DiagonalPose::Result diagonal_pose;
    };
    static PillarDiagnostics &instance() { static PillarDiagnostics value; return value; }
    void begin() { enabled_ = false; size_ = total_ = straight_ = diagonal_ = rejected_ = applied_ = 0; enabled_ = true; }
    void stop() { enabled_ = false; }
    unsigned size() const { return size_; }
    const Event &event(unsigned index) const { return events_[index]; }
    static bool front_pair_close(const Sample &s) {
        return s.range[2] == IrRangeState::InRange && s.range[3] == IrRangeState::InRange
            && s.distance[2] <= 66 && s.distance[3] <= 66;
    }
    static bool diagonal_front_pair_blocked(const Sample &s) {
        return ir_range_can_see_wall(s.range[2]) && ir_range_can_see_wall(s.range[3])
            && s.distance[2]<=66 && s.distance[3]<=66;
    }
    static uint16_t quality_flags(const SpatialPillarPeak::Event &peak, const Sample &minimum,
        const Sample &confirmation, uint8_t axis, uint8_t side) {
        uint16_t flags = peak.context_flags | (peak.motion_clean ? 0 : Rotating);
        const Sample *samples[] = {&minimum, &confirmation};
        for (const Sample *sample : samples) {
            if (!isfinite(sample->length_mm) || !isfinite(sample->left_mm) || !isfinite(sample->heading_rad)
                || !isfinite(sample->gyro_radps) || !isfinite(sample->control_left_mm) || !isfinite(sample->control_heading_rad)) flags |= InvalidPose;
        }
        if (minimum.generation != confirmation.generation) flags |= FrameChanged;
        const float jump = confirmation.length_mm - minimum.length_mm - peak.sample_delay_mm;
        if (!isfinite(jump) || fabsf(jump) > 1.0f) flags |= PositionJump;
        if (front_pair_close(minimum) || front_pair_close(confirmation)) flags |= FrontPairClose;
        const unsigned sensor = side == 2U ? 1U : 0U;
        // A clipped peak is an interval/bound, including straight Open->Open.
        // Keep its diagnostic record, but do not inject its midpoint as a calibrated point.
        if (minimum.range[sensor] == IrRangeState::NearLimit
            || (axis==0 && minimum.distance[sensor]<=IR_SIDE_MIN_MM && peak.width_mm>0)) flags |= SideNearLimit;
        if (minimum.range[2] == IrRangeState::Invalid || minimum.range[3] == IrRangeState::Invalid
            || confirmation.range[2] == IrRangeState::Invalid || confirmation.range[3] == IrRangeState::Invalid) flags |= InvalidFront;
        return flags;
    }
    int record(const SpatialPillarPeak::Event &peak, const Sample &minimum, const Sample &confirmation,
        uint8_t axis, uint8_t side, uint8_t motion, uint8_t control, uint8_t turn,
        WallPositionCorrection::Mode mode, bool feature_frame,
        float reference_mm = wall_feature_ekf_param.pillar_reference_mm,
        WallPositionCorrection::Phase phase = WallPositionCorrection::Phase::SearchStraight) {
        if (!enabled_ || !peak.detected) return -1;
        ++total_; if (axis == 0) ++straight_; else ++diagonal_;
        const uint16_t flags = quality_flags(peak,minimum,confirmation,axis,side);
        const float jump = confirmation.length_mm - minimum.length_mm - peak.sample_delay_mm;
        if (flags) ++rejected_;
        if (size_ >= capacity) return -1;
        Event &out = events_[size_++]; out = Event{};
        out.minimum = minimum; out.confirmation = confirmation; out.peak = peak;
        out.axis = axis; out.side = side; out.motion = motion; out.control = control; out.turn = turn; out.mode = mode;
        out.flags = flags; out.coordinate_jump_mm = jump;
        out.phase = phase;
        out.feature_mm = minimum.length_mm + peak.sample_delay_mm - peak.delay_mm;
        out.reference_valid = wall_feature_ekf_param.enabled && feature_frame && axis == 0 && !flags && isfinite(reference_mm);
        if (out.reference_valid) {
            out.reference_mm = reference_mm; out.target_mm = reference_mm + peak.delay_mm;
            out.innovation_mm = out.target_mm - confirmation.length_mm;
            const auto &policy=wall_position_param[static_cast<unsigned>(phase)];
            // This is a preview; position_result records the actual EKF decision.
            if (WallPositionCorrection::evaluate(confirmation.length_mm,reference_mm,peak.delay_mm,phase,
                    straight_pillar_param.maximum_after_peak_mm,true).accepted()) {
                out.proposal_valid = true;
                const float blend=fminf(policy.blend,wall_feature_ekf_param.maximum_gain);
                out.proposed_delta_mm = fmaxf(-policy.maximum_step_mm, fminf(policy.maximum_step_mm, blend*out.innovation_mm));
            }
        }
        return static_cast<int>(size_-1);
    }
    void rebase_pending(int index, float delta_mm, bool move_reference = true) {
        if (index < 0 || static_cast<unsigned>(index) >= size_) return;
        auto &e=events_[index];
        if (e.position_evaluated) return;
        e.minimum.length_mm+=delta_mm; e.minimum.rebase_mm+=delta_mm;
        e.confirmation.length_mm+=delta_mm; e.feature_mm+=delta_mm;
        if (e.reference_valid && move_reference) { e.reference_mm+=delta_mm; e.target_mm+=delta_mm; }
    }
    void continue_pending(int index, uint32_t generation) {
        if (index < 0 || static_cast<unsigned>(index) >= size_) return;
        auto &e=events_[index];
        if (!e.position_evaluated) e.minimum.generation=e.confirmation.generation=generation;
    }
    void position_result(int index, const WallPositionCorrection::Result &result) {
        if (enabled_ && result.accepted()) ++applied_;
        if (index < 0 || static_cast<unsigned>(index) >= size_) return;
        auto &event=events_[index]; event.position_evaluated = true; event.position = result;
        event.proposal_valid=result.proposed_reason==WallPositionCorrection::Reason::Accepted || result.proposed_reason==WallPositionCorrection::Reason::Limited;
        event.target_mm=result.target_mm; event.innovation_mm=result.innovation_mm; event.proposed_delta_mm=result.proposed_delta_mm;
    }
    void diagonal_shadow_result(int index, const DiagonalFeatureShadow::Decision &decision) {
        position_result(index,decision.position);
        if (index<0 || static_cast<unsigned>(index)>=size_) return;
        auto &e=events_[index]; e.diagonal_model=decision.model;
        e.reference_valid=decision.model.reference_valid && !e.flags;
        e.reference_mm=e.reference_valid ? decision.position.target_mm-decision.position.delay_mm : 0;
    }
    void diagonal_pose_result(int index, const DiagonalPose::Result &result) {
        if (index>=0 && static_cast<unsigned>(index)<size_) events_[index].diagonal_pose=result;
    }
    static void print_diagonal_model(const DiagonalFeatureShadow::Metadata &m, bool end_line=true) {
        printf(" position_shadow_only:%u position_model_ready:%u position_status:%s model_reference_mm:%.3f model_pose_shift_mm:%.3f model_e_mm:%.3f model_psi_rad:%.6f position_quality_flags:%u",
            m.shadow_only?1U:0U,m.model_ready?1U:0U,DiagonalFeatureShadow::status_name(m.status),
            m.nominal_reference_mm,m.pose_shift_mm,m.left_mm,m.heading_rad,m.quality_flags);
        if (end_line) printf("\r\n");
    }
    void finish_and_print() {
        enabled_ = false;
        printf("PILLAR_STATS schema:1 total:%lu straight:%lu diagonal:%lu quality_rejected:%lu stored:%u dropped:%lu detector:1 applied:%lu st_open_mm:%.3f st_open_travel_mm:%.3f st_prominence_adc:%.3f st_confirm_adc:%.3f st_top_adc:%.3f st_separation_mm:%.3f di_near_mm:%.3f di_prominence_mm:%.3f di_confirm_mm:%.3f di_top_mm:%.3f di_separation_mm:%.3f\r\n",
            (unsigned long)total_, (unsigned long)straight_, (unsigned long)diagonal_, (unsigned long)rejected_, size_, (unsigned long)(total_-size_),
            (unsigned long)applied_,straight_pillar_param.open_mm,straight_pillar_param.open_travel_mm,straight_pillar_param.approach_signal,
            straight_pillar_param.confirm_signal,straight_pillar_param.top_tolerance,straight_pillar_param.separation_mm,
            diagonal_pillar_param.near_mm,diagonal_pillar_param.approach_signal,diagonal_pillar_param.confirm_signal,
            diagonal_pillar_param.top_tolerance,diagonal_pillar_param.separation_mm);
        for (unsigned i=0; i<size_; ++i) {
            const auto &e=events_[i]; const auto &m=e.minimum; const auto &c=e.confirmation;
            printf("PILLAR_EDGE index:%u ms:%lu minimum_ms:%lu axis:%u side:%u motion:%u control:%u turn:%u feature:%s mode:%s reason:%s flags:%u generation:%lu event_generation:%lu feature_mm:%.3f delay_mm:%.3f sample_delay_mm:%.3f width_mm:%.3f prominence:%.3f before_mm:%.3f after_mm:%.3f delta_mm:%.3f reference_valid:%u proposal_valid:%u reference_mm:%.3f target_mm:%.3f innovation_mm:%.3f proposed_delta_mm:%.3f coordinate_jump_mm:%.3f minimum_length_mm:%.3f e_mm:%.3f psi_rad:%.6f gyro_radps:%.4f control_ir:%.3f control_ir_dot:%.6f confirm_e_mm:%.3f confirm_psi_rad:%.6f confirm_gyro_radps:%.4f sl_mm:%.3f sr_mm:%.3f fl_mm:%.3f fr_mm:%.3f confirm_fl_mm:%.3f confirm_fr_mm:%.3f adc_l:%d adc_r:%d adc_fl:%d adc_fr:%d range_l:%u range_r:%u range_fl:%u range_fr:%u position_evaluated:%u position_reason:%s position_phase:%u minimum_source_generation:%lu minimum_rebase_mm:%.3f ekf:%u ekf_nis:%.3f ekf_gain:%.6f ekf_variance_mm2:%.5f",
                i,(unsigned long)c.ms,(unsigned long)m.ms,e.axis,e.side,e.motion,e.control,e.turn,
                e.axis ? "diagonal_pillar_minimum" : "straight_pillar_adc_maximum", WallPositionCorrection::mode_name(e.mode),
                e.flags ? "quality_rejected" : e.position_evaluated ? WallPositionCorrection::reason_name(e.position.reason) : "shadow", e.flags,(unsigned long)c.generation,(unsigned long)m.generation,
                e.feature_mm,e.peak.delay_mm,e.peak.sample_delay_mm,e.peak.width_mm,e.peak.prominence,c.length_mm,e.position_evaluated?e.position.after_mm:c.length_mm,e.position_evaluated?e.position.delta_mm:0.0f,
                e.reference_valid?1U:0U,e.proposal_valid?1U:0U,e.reference_mm,e.target_mm,e.innovation_mm,e.proposed_delta_mm,e.coordinate_jump_mm,
                m.length_mm,m.left_mm,m.heading_rad,m.gyro_radps,m.control_left_mm,m.control_heading_rad,
                c.left_mm,c.heading_rad,c.gyro_radps,m.distance[0],m.distance[1],m.distance[2],m.distance[3],c.distance[2],c.distance[3],
                m.adc[0],m.adc[1],m.adc[2],m.adc[3],(unsigned)m.range[0],(unsigned)m.range[1],(unsigned)m.range[2],(unsigned)m.range[3],e.position_evaluated?1U:0U,WallPositionCorrection::reason_name(e.position.reason),static_cast<unsigned>(e.phase),(unsigned long)m.source_generation,m.rebase_mm,e.position.ekf?1U:0U,e.position.ekf_nis,e.position.ekf_gain,e.position.ekf_variance_mm2);
            print_diagonal_model(e.diagonal_model,false);
            const auto &p=e.diagonal_pose;
            if (p.status!=DiagonalPose::NoRoute) {
                printf(" diag_pose_status:%u diag_pose_pillar:%u diag_pose_used:%u diag_pose_ready:%u diag_pose_s_mm:%.3f diag_pose_e_mm:%.3f diag_pose_heading_rad:%.6f diag_pose_sigma_s_mm:%.3f diag_pose_sigma_e_mm:%.3f diag_pose_sigma_heading_rad:%.6f diag_pose_innovation_mm:%.3f diag_pose_nis:%.4f",
                    static_cast<unsigned>(p.status),p.index,p.used,p.lateral_heading_ready?1U:0U,p.pose.s,p.pose.e,p.pose.heading,
                    p.sigma_s,p.sigma_e,p.sigma_heading,p.innovation,p.nis);
            }
            printf("\r\n");
        }
    }
private:
    volatile bool enabled_ = false;
    unsigned size_ = 0;
    uint32_t total_ = 0, straight_ = 0, diagonal_ = 0, rejected_ = 0, applied_ = 0;
    Event events_[capacity]{};
};

// 壁切れ補正と範囲状態の記録
// Only correction decisions are stored, not a continuous IRQ trace. A single
// control IRQ writes; begin/finish run in foreground with motion stopped.
// No allocation, communication or waits on the recording path.
class WallDiagnostics {
public:
    static constexpr unsigned capacity = 64;
    struct Event {
        uint32_t time_ms;
        WallPositionCorrection::Result correction;
        int16_t adc[4]; // L, R, FL, FR
        IrRangeState range[4];
        uint8_t motion, phase, side;
        WallPositionCorrection::Mode mode;
        uint32_t generation, event_generation;
        DiagonalFeatureShadow::Metadata diagonal_model;
    };
    static WallDiagnostics &instance() { static WallDiagnostics value; return value; }
    void begin(WallPositionCorrection::Mode mode = WallPositionCorrection::Mode::Legacy) {
        PillarDiagnostics::instance().begin();
        enabled_ = false;
        size_ = total_ = all_samples_ = moving_samples_ = 0;
        for (auto &count : decisions_) count = 0;
        for (auto &sensor : ranges_) for (auto &count : sensor) count = 0;
        mode_ = mode;
        enabled_ = true;
    }
    bool enabled() const { return enabled_; }
    void stop() { enabled_ = false; PillarDiagnostics::instance().stop(); }
    void sample(IrRangeState left, IrRangeState right, IrRangeState front_left, IrRangeState front_right, bool moving) {
        if (!enabled_) return;
        ++all_samples_;
        if (!moving) return;
        ++moving_samples_;
        const IrRangeState states[] = {left, right, front_left, front_right};
        for (unsigned i = 0; i < 4; ++i) {
            unsigned value = static_cast<unsigned>(states[i]);
            ++ranges_[i][value < 4 ? value : 3];
        }
    }
    void record(uint32_t ms, uint8_t motion, WallPositionCorrection::Phase phase, uint8_t side,
                const WallPositionCorrection::Result &correction, const int16_t *adc, const IrRangeState *states,
                WallPositionCorrection::Mode mode = WallPositionCorrection::Mode::Legacy,
                uint32_t generation = 0, uint32_t event_generation = 0,
                const DiagonalFeatureShadow::Metadata &model = DiagonalFeatureShadow::Metadata{}) {
        if (!enabled_) return;
        ++total_;
        ++decisions_[static_cast<unsigned>(correction.reason)];
        if (size_ >= capacity) return;
        Event &event = events_[size_++];
        event.time_ms = ms; event.motion = motion; event.phase = static_cast<uint8_t>(phase); event.side = side;
        event.correction = correction;
        event.mode = mode; event.generation = generation; event.event_generation = event_generation;
        event.diagonal_model=model;
        for (unsigned i = 0; i < 4; ++i) { event.adc[i] = adc[i]; event.range[i] = states[i]; }
    }
    unsigned size() const { return size_; }
    uint32_t total() const { return total_; }
    const Event &event(unsigned index) const { return events_[index]; }
    void finish_and_print() {
        enabled_ = false;
        printf("WALL_RANGE_STATS schema:1 all_samples:%lu moving_samples:%lu",
               static_cast<unsigned long>(all_samples_), static_cast<unsigned long>(moving_samples_));
        const char *sensors[] = {"l", "r", "fl", "fr"};
        const char *states[] = {"ok", "near", "far", "invalid"};
        for (unsigned s = 0; s < 4; ++s) for (unsigned q = 0; q < 4; ++q)
            printf(" %s_%s:%lu", sensors[s], states[q], static_cast<unsigned long>(ranges_[s][q]));
        printf("\r\nWALL_CORR_STATS schema:1 total:%lu accepted:%lu limited:%lu rejected:%lu invalid:%lu wrong_side:%lu stored:%u dropped:%lu shadow:%lu not_selected:%lu stale_generation:%lu mode:%s arm_range_mm:%.3f arm_travel_mm:%.3f search_reference_mm:%.3f",
            static_cast<unsigned long>(total_), static_cast<unsigned long>(decisions_[0]),
            static_cast<unsigned long>(decisions_[1]), static_cast<unsigned long>(decisions_[2]),
            static_cast<unsigned long>(decisions_[3]), static_cast<unsigned long>(decisions_[4]), size_, static_cast<unsigned long>(total_ - size_),
            static_cast<unsigned long>(decisions_[5]), static_cast<unsigned long>(decisions_[6]), static_cast<unsigned long>(decisions_[7]),
            WallPositionCorrection::mode_name(mode_), WallPositionCorrection::edge_param(mode_).arm_range_mm,
            WallPositionCorrection::edge_param(mode_).arm_travel_mm, WallPositionCorrection::search_reference(mode_));
        for (unsigned p = 0; p < 7; ++p) {
            const auto &policy = wall_position_param[p];
            printf(" p%u_window_mm:%.3f p%u_max_step_mm:%.3f p%u_blend:%.3f",
                p, policy.window_mm, p, policy.maximum_step_mm, p, policy.blend);
        }
        printf("\r\n");
        const char *reasons[] = {"accepted", "limited", "outside_window", "invalid", "wrong_side", "shadow", "not_selected", "stale_generation"};
        for (unsigned i = 0; i < size_; ++i) {
            const auto &e = events_[i]; const auto &c = e.correction;
            printf("WALL_CORR index:%u ms:%lu motion:%u phase:%u side:%u reason:%s before_mm:%.3f target_mm:%.3f delay_mm:%.3f innovation_mm:%.3f delta_mm:%.3f after_mm:%.3f adc_l:%d adc_r:%d adc_fl:%d adc_fr:%d range_l:%u range_r:%u range_fl:%u range_fr:%u mode:%s proposed_reason:%s proposed_delta_mm:%.3f proposed_after_mm:%.3f generation:%lu event_generation:%lu feature:%s ekf:%u ekf_nis:%.3f ekf_gain:%.6f ekf_variance_mm2:%.5f",
                i, static_cast<unsigned long>(e.time_ms), e.motion, e.phase, e.side,
                reasons[static_cast<unsigned>(c.reason)], c.before_mm, c.target_mm, c.delay_mm,
                c.innovation_mm, c.delta_mm, c.after_mm, e.adc[0], e.adc[1], e.adc[2], e.adc[3],
                static_cast<unsigned>(e.range[0]), static_cast<unsigned>(e.range[1]),
                static_cast<unsigned>(e.range[2]), static_cast<unsigned>(e.range[3]), WallPositionCorrection::mode_name(e.mode),
                reasons[static_cast<unsigned>(c.proposed_reason)], c.proposed_delta_mm, c.proposed_after_mm,
                static_cast<unsigned long>(e.generation), static_cast<unsigned long>(e.event_generation),
                WallPositionCorrection::diagonal_phase(static_cast<WallPositionCorrection::Phase>(e.phase)) ? "diagonal_difference_crossing" : "wall_edge_crossing",c.ekf?1U:0U,c.ekf_nis,c.ekf_gain,c.ekf_variance_mm2);
            PillarDiagnostics::print_diagonal_model(e.diagonal_model);
        }
        PillarDiagnostics::instance().finish_and_print();
    }
private:
    volatile bool enabled_ = false;
    unsigned size_ = 0;
    uint32_t total_ = 0, all_samples_ = 0, moving_samples_ = 0;
    WallPositionCorrection::Mode mode_ = WallPositionCorrection::Mode::Legacy;
    uint32_t decisions_[8] = {}, ranges_[4][4] = {};
    Event events_[capacity]{};
};

#endif
