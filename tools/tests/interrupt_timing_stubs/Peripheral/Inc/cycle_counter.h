#ifndef TEST_CYCLE_COUNTER_H_
#define TEST_CYCLE_COUNTER_H_
#include <stdint.h>
namespace CycleCounter
{
struct State { bool supported; uint32_t hz, cycles; };
inline State &state() { static State value = {true, 64000000U, 0U}; return value; }
inline bool enable() { return state().supported; }
inline uint32_t frequency() { return state().hz; }
inline uint32_t read() { return state().cycles; }
}
#endif
