#ifndef WALL_POSITION_CORRECTION_H_
#define WALL_POSITION_CORRECTION_H_

#include <math.h>
#include <stdint.h>
#include "Params/wall_param.h"

namespace WallPositionCorrection {
enum class Phase : uint8_t { StraightEntry, DiagonalEntry, StraightExit, DiagonalExit, SearchStraight };
enum class Reason : uint8_t { Accepted, Limited, OutsideWindow, Invalid, WrongSide };
struct Result {
    float before_mm = 0.0f, target_mm = 0.0f, after_mm = 0.0f;
    float innovation_mm = 0.0f, delta_mm = 0.0f, delay_mm = 0.0f;
    Reason reason = Reason::Invalid;
    bool accepted() const { return reason == Reason::Accepted || reason == Reason::Limited; }
};
inline Result evaluate(float before_mm, float edge_reference_mm, float delay_mm, Phase phase)
{
    Result out;
    out.before_mm = out.after_mm = before_mm; out.delay_mm = delay_mm;
    const unsigned index = static_cast<unsigned>(phase);
    if (index >= 5) return out;
    const auto &policy = wall_position_param[index];
    out.target_mm = edge_reference_mm + delay_mm;
    out.innovation_mm = out.target_mm - before_mm;
    const float maximum_delay = (index & 1U) ? diagonal_edge_param.maximum_pending_mm : wall_edge_param.maximum_pending_mm;
    if (!isfinite(before_mm) || !isfinite(out.target_mm) || !isfinite(delay_mm)
        || delay_mm < 0.0f || delay_mm > maximum_delay
        || !isfinite(policy.window_mm) || !isfinite(policy.maximum_step_mm) || !isfinite(policy.blend)
        || policy.window_mm <= 0.0f || policy.maximum_step_mm <= 0.0f || policy.blend <= 0.0f || policy.blend > 1.0f)
        return out;
    if (fabsf(out.innovation_mm) > policy.window_mm) { out.reason = Reason::OutsideWindow; return out; }
    const float desired = out.innovation_mm * policy.blend;
    out.delta_mm = fmaxf(-policy.maximum_step_mm, fminf(policy.maximum_step_mm, desired));
    out.after_mm = before_mm + out.delta_mm;
    out.reason = out.delta_mm == desired ? Reason::Accepted : Reason::Limited;
    return out;
}
}

#endif
