#ifndef WALL_DIAGNOSTICS_H_
#define WALL_DIAGNOSTICS_H_

#include <stdint.h>
#include <stdio.h>
#include "Task/Inc/ir_range_state.h"
#include "Task/Inc/wall_position_correction.h"

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
    };
    static WallDiagnostics &instance() { static WallDiagnostics value; return value; }
    void begin() {
        enabled_ = false;
        size_ = total_ = all_samples_ = moving_samples_ = 0;
        for (auto &count : decisions_) count = 0;
        for (auto &sensor : ranges_) for (auto &count : sensor) count = 0;
        enabled_ = true;
    }
    bool enabled() const { return enabled_; }
    void stop() { enabled_ = false; }
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
                const WallPositionCorrection::Result &correction, const int16_t *adc, const IrRangeState *states) {
        if (!enabled_) return;
        ++total_;
        ++decisions_[static_cast<unsigned>(correction.reason)];
        if (size_ >= capacity) return;
        Event &event = events_[size_++];
        event.time_ms = ms; event.motion = motion; event.phase = static_cast<uint8_t>(phase); event.side = side;
        event.correction = correction;
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
        printf("\r\nWALL_CORR_STATS schema:1 total:%lu accepted:%lu limited:%lu rejected:%lu invalid:%lu wrong_side:%lu stored:%u dropped:%lu",
            static_cast<unsigned long>(total_), static_cast<unsigned long>(decisions_[0]),
            static_cast<unsigned long>(decisions_[1]), static_cast<unsigned long>(decisions_[2]),
            static_cast<unsigned long>(decisions_[3]), static_cast<unsigned long>(decisions_[4]), size_, static_cast<unsigned long>(total_ - size_));
        for (unsigned p = 0; p < 5; ++p) {
            const auto &policy = wall_position_param[p];
            printf(" p%u_window_mm:%.3f p%u_max_step_mm:%.3f p%u_blend:%.3f",
                p, policy.window_mm, p, policy.maximum_step_mm, p, policy.blend);
        }
        printf("\r\n");
        const char *reasons[] = {"accepted", "limited", "outside_window", "invalid", "wrong_side"};
        for (unsigned i = 0; i < size_; ++i) {
            const auto &e = events_[i]; const auto &c = e.correction;
            printf("WALL_CORR index:%u ms:%lu motion:%u phase:%u side:%u reason:%s before_mm:%.3f target_mm:%.3f delay_mm:%.3f innovation_mm:%.3f delta_mm:%.3f after_mm:%.3f adc_l:%d adc_r:%d adc_fl:%d adc_fr:%d range_l:%u range_r:%u range_fl:%u range_fr:%u\r\n",
                i, static_cast<unsigned long>(e.time_ms), e.motion, e.phase, e.side,
                reasons[static_cast<unsigned>(c.reason)], c.before_mm, c.target_mm, c.delay_mm,
                c.innovation_mm, c.delta_mm, c.after_mm, e.adc[0], e.adc[1], e.adc[2], e.adc[3],
                static_cast<unsigned>(e.range[0]), static_cast<unsigned>(e.range[1]),
                static_cast<unsigned>(e.range[2]), static_cast<unsigned>(e.range[3]));
        }
    }
private:
    volatile bool enabled_ = false;
    unsigned size_ = 0;
    uint32_t total_ = 0, all_samples_ = 0, moving_samples_ = 0;
    uint32_t decisions_[5] = {}, ranges_[4][4] = {};
    Event events_[capacity]{};
};

#endif
