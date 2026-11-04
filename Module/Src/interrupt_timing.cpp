#include "Module/Inc/interrupt_timing.h"
#include "Peripheral/Inc/cycle_counter.h"
#include "Core/Inc/main.h"
#include <math.h>

namespace
{
bool ready, valid, previous_valid, period_valid;
float us_per_cycle;
uint32_t previous_start, pending_logging;
uint32_t total_cycles, logging_cycles, max_cycles, period_cycles;
}

void InterruptTiming::initialize()
{
	const uint32_t mask = __get_PRIMASK();
	__disable_irq();
	ready = CycleCounter::enable() && CycleCounter::frequency() != 0U;
	us_per_cycle = ready ? 1000000.0f / static_cast<float>(CycleCounter::frequency()) : 0.0f;
	reset();
	__set_PRIMASK(mask);
}

bool InterruptTiming::available() { return ready; }
uint32_t InterruptTiming::now() { return ready ? CycleCounter::read() : 0U; }
void InterruptTiming::set_logging_cycles(uint32_t elapsed) { pending_logging = elapsed; }

void InterruptTiming::record(uint32_t entered, uint32_t finished)
{
	if(!ready) return;
	total_cycles = finished - entered;
	logging_cycles = pending_logging;
	if(total_cycles > max_cycles) max_cycles = total_cycles;
	period_valid = previous_valid;
	if(previous_valid) period_cycles = entered - previous_start;
	previous_start = entered;
	previous_valid = valid = true;
}

void InterruptTiming::reset()
{
	const uint32_t mask = __get_PRIMASK();
	__disable_irq();
	valid = previous_valid = period_valid = false;
	previous_start = pending_logging = 0;
	total_cycles = logging_cycles = max_cycles = period_cycles = 0;
	__set_PRIMASK(mask);
}

InterruptTiming::Sample InterruptTiming::sample()
{
	if(!ready || !valid) return {NAN, NAN, NAN, NAN};
	return {total_cycles * us_per_cycle, logging_cycles * us_per_cycle,
		max_cycles * us_per_cycle, period_valid ? period_cycles * us_per_cycle : NAN};
}
