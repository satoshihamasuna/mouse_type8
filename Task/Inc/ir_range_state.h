#ifndef IR_RANGE_STATE_H_
#define IR_RANGE_STATE_H_

#include <stdint.h>

// Distance values retain the legacy clamped mm representation. State comes
// from ADC endpoints BEFORE interpolation; an exact calibrated endpoint is valid.
enum class IrRangeState : uint8_t { InRange = 0, NearLimit = 1, FarLimit = 2, Invalid = 3, Unknown = 4 };
constexpr float IR_SIDE_MIN_MM = 25.0f;

inline IrRangeState ir_range_state(int16_t adc, const int16_t *table, int count)
{
    if (!table || count < 2 || adc < 0) return IrRangeState::Invalid;
    if (adc > table[0]) return IrRangeState::NearLimit;
    if (adc < table[count - 1]) return IrRangeState::FarLimit;
    return IrRangeState::InRange;
}

inline bool ir_range_is_metric(IrRangeState state) { return state == IrRangeState::InRange; }
inline bool ir_range_can_see_wall(IrRangeState state)
{
    return state == IrRangeState::InRange || state == IrRangeState::NearLimit;
}

#endif
