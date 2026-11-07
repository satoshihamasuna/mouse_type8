#ifndef SEARCH_TURN_DIAGNOSTICS_H_
#define SEARCH_TURN_DIAGNOSTICS_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "Task/Inc/search_turn_guard.h"

// Written only by the foreground Search decisions, dumped after motors stop.
// Keep the first decisions for repeatable route-prefix comparisons; count all.
class SearchTurnDiagnostics {
public:
    static constexpr unsigned capacity = 64;
    struct Event {
        uint32_t time_ms;
        float left_mm, right_mm, front_left_mm, front_right_mm;
        float lateral_mm, heading_rad, lateral_variance_mm2, heading_variance_rad2;
        uint8_t x, y, heading, right_turn, reason, flags;
    };
    static SearchTurnDiagnostics &instance() { static SearchTurnDiagnostics value; return value; }
    void begin() {
        total_ = suppressed_ = 0; size_ = 0;
        for (auto &count : counts_) count = 0;
        enabled_ = true;
    }
    void record(uint32_t ms, uint8_t x, uint8_t y, uint8_t heading, const SearchTurnGuard::Decision &decision) {
        if (!enabled_) return;
        ++total_; ++counts_[decision.reason];
        if (decision.flags & SearchTurnGuard::Suppressed) ++suppressed_;
        if (size_ >= capacity) return;
        const auto &in = decision.input;
        events_[size_++] = {ms, in.left_mm, in.right_mm, in.front_left_mm, in.front_right_mm,
            in.lateral_mm, in.heading_rad, in.lateral_variance_mm2, in.heading_variance_rad2,
            x, y, heading, static_cast<uint8_t>(in.right_turn), static_cast<uint8_t>(decision.reason), decision.flags};
    }
    uint32_t total() const { return total_; }
    unsigned size() const { return size_; }
    uint32_t count(SearchTurnGuard::Reason reason) const { return counts_[reason]; }
    uint32_t suppressed() const { return suppressed_; }
    void finish_and_print() {
        enabled_ = false;
        printf("EXPLORE_TURN_STATS total:%lu normal:%lu side:%lu front_diff:%lu front_near:%lu suppressed:%lu stored:%u dropped:%lu\r\n",
            static_cast<unsigned long>(total_), static_cast<unsigned long>(counts_[0]),
            static_cast<unsigned long>(counts_[1]), static_cast<unsigned long>(counts_[2]),
            static_cast<unsigned long>(counts_[3]), static_cast<unsigned long>(suppressed_), size_,
            static_cast<unsigned long>(total_ - size_));
        const char *names[] = {"normal", "side", "front_diff", "front_near"};
        for (unsigned i = 0; i < size_; ++i) {
            const auto &e = events_[i];
            printf("EXPLORE_TURN index:%u ms:%lu x:%u y:%u heading:%u turn:%s reason:%s flags:%u sl_mm:%.3f sr_mm:%.3f fl_mm:%.3f fr_mm:%.3f e_mm:%.3f psi_rad:%.6f sigma_e_mm:%.3f sigma_psi_rad:%.6f\r\n",
                i, static_cast<unsigned long>(e.time_ms), e.x, e.y, e.heading, e.right_turn ? "R" : "L", names[e.reason], e.flags,
                static_cast<double>(e.left_mm), static_cast<double>(e.right_mm),
                static_cast<double>(e.front_left_mm), static_cast<double>(e.front_right_mm),
                static_cast<double>(e.lateral_mm), static_cast<double>(e.heading_rad),
                static_cast<double>(sqrtf(fmaxf(0.0f, e.lateral_variance_mm2))),
                static_cast<double>(sqrtf(fmaxf(0.0f, e.heading_variance_rad2))));
        }
    }
private:
    Event events_[capacity]{};
    uint32_t total_ = 0, counts_[4]{}, suppressed_ = 0;
    unsigned size_ = 0;
    bool enabled_ = false;
};
#endif
