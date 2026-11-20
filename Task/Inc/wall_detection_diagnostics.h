#ifndef TASK_INC_WALL_DETECTION_DIAGNOSTICS_H_
#define TASK_INC_WALL_DETECTION_DIAGNOSTICS_H_

#include <stdint.h>
#include <stdio.h>
#include "Task/Inc/ir_range_state.h"
#include "Task/Inc/run_typedef.h"

// 補正の採否・使用済みラッチとは独立した、検出器の出力記録。
// IRQでは固定長メモリへの保存だけを行い、停止後に出力する。
class WallDetectionDiagnostics {
public:
    static constexpr unsigned capacity = 64;
    enum Phase : uint8_t { Straight, Diagonal, Before, Turning, After, Other };
    struct Sample {
        uint32_t ms = 0, generation = 0;
        float length_mm = 0, gyro_radps = 0, distance[4] = {};
        int16_t adc[4] = {};
        IrRangeState range[4] = {};
        uint8_t motion = 0, control = 0, turn = 0, turn_side = 0, axis = 0;
        Phase phase = Straight;
    };
    struct Event {
        Sample sample;
        uint32_t segment = 0, start_ms = 0, samples = 0, left = 0, right = 0;
        float delay_mm = 0;
        uint16_t age_ticks = 0;
        uint8_t side = 0; // 0=区間終了、1=左検出、2=右検出
        bool stopped = false;
    };
    static WallDetectionDiagnostics &instance() { static WallDetectionDiagnostics value; return value; }
    void begin() {
        enabled_ = false; active_ = false;
        size_ = total_ = edges_ = phases_ = segment_ = 0;
        enabled_ = true;
    }
    bool enabled() const { return enabled_; }
    unsigned size() const { return size_; }
    uint32_t total() const { return total_; }
    const Event &event(unsigned index) const { return events_[index]; }
    static Phase phase(uint8_t motion, uint8_t control, uint8_t turn) {
        const bool turning = (motion >= Long_turnR90 && motion <= Long_turn_LV90)
            || motion == Search_slalom_R || motion == Search_slalom_L;
        if (turning) return turn == Prev_Turn ? Before : turn == Post_Turn ? After : Turning;
        return control == STRAIGHT_STATE ? Straight : control == DIAGONAL_STATE ? Diagonal : Other;
    }
    static const char *phase_name(Phase value) {
        const char *names[] = {"straight", "diagonal", "before", "turning", "after", "other"};
        return names[static_cast<unsigned>(value)];
    }
    void update(const Sample &sample, uint8_t mask, float left_delay, float right_delay,
                uint16_t left_age, uint16_t right_age) {
        if (!enabled_) return;
        if (!active_ || sample.motion != current_.sample.motion || sample.phase != current_.sample.phase
            || sample.turn_side != current_.sample.turn_side) {
            close(false);
            current_ = Event{}; current_.segment = segment_++;
            current_.start_ms = sample.ms; active_ = true;
        }
        current_.sample = sample; ++current_.samples;
        if (mask & 1U) ++current_.left;
        if (mask & 2U) ++current_.right;
        for (unsigned side = 1; side <= 2; ++side) if (mask & side) {
            Event edge = current_; edge.side = side;
            edge.delay_mm = side == 1 ? left_delay : right_delay;
            edge.age_ticks = side == 1 ? left_age : right_age;
            ++edges_; append(edge);
        }
    }
    void stop() { if (enabled_) close(true); enabled_ = false; }
    void finish_and_print() {
        stop();
        printf("WALL_DETECT_STATS schema:1 total:%lu edges:%lu phases:%lu stored:%u dropped:%lu\r\n",
            static_cast<unsigned long>(total_), static_cast<unsigned long>(edges_),
            static_cast<unsigned long>(phases_), size_, static_cast<unsigned long>(total_ - size_));
        for (unsigned i = 0; i < size_; ++i) {
            const auto &e = events_[i]; const auto &s = e.sample;
            const unsigned expected = s.phase == Before ? s.turn_side : 0;
            const int seen = expected == 1 ? (e.left > 0) : expected == 2 ? (e.right > 0) : -1;
            printf("WALL_DETECT index:%u kind:%s segment:%lu start_ms:%lu ms:%lu motion:%u control:%u turn:%u phase:%s axis:%u turn_side:%u expected_side:%u expected_seen:%d side:%u left_count:%lu right_count:%lu samples:%lu end:%s generation:%lu length_mm:%.3f delay_mm:%.3f age_ticks:%u gyro_radps:%.5f sl_mm:%.3f sr_mm:%.3f fl_mm:%.3f fr_mm:%.3f adc_l:%d adc_r:%d adc_fl:%d adc_fr:%d range_l:%u range_r:%u range_fl:%u range_fr:%u",
                i, e.side ? "edge" : "phase_end", static_cast<unsigned long>(e.segment),
                static_cast<unsigned long>(e.start_ms), static_cast<unsigned long>(s.ms), s.motion, s.control, s.turn,
                phase_name(s.phase), s.axis, s.turn_side, expected, seen, e.side,
                static_cast<unsigned long>(e.left), static_cast<unsigned long>(e.right), static_cast<unsigned long>(e.samples),
                e.side ? "pending" : e.stopped ? "stop" : "transition", static_cast<unsigned long>(s.generation),
                s.length_mm, e.delay_mm, e.age_ticks, s.gyro_radps, s.distance[0], s.distance[1], s.distance[2], s.distance[3],
                s.adc[0], s.adc[1], s.adc[2], s.adc[3], static_cast<unsigned>(s.range[0]), static_cast<unsigned>(s.range[1]),
                static_cast<unsigned>(s.range[2]), static_cast<unsigned>(s.range[3]));
            if (e.side) printf(" feature_mm:%.3f", s.length_mm-e.delay_mm);
            printf("\r\n");
        }
    }
private:
    void append(const Event &event) { ++total_; if (size_ < capacity) events_[size_++] = event; }
    void close(bool stopped) {
        if (active_ && (current_.sample.phase == Before || current_.sample.phase == After)) {
            current_.stopped = stopped; ++phases_; append(current_);
        }
        active_ = false;
    }
    bool enabled_ = false, active_ = false;
    unsigned size_ = 0;
    uint32_t total_ = 0, edges_ = 0, phases_ = 0, segment_ = 0;
    Event current_;
    // 既定値はすべてゼロ。配列は.bssへ置き、Flashの.dataを増やさない。
    Event events_[capacity]{};
};

#endif
