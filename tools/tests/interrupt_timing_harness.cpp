#include "Module/Inc/interrupt_timing.h"
#include "Peripheral/Inc/cycle_counter.h"
#include <cassert>
#include <cmath>

uint32_t test_primask;

static void unavailable()
{
	const auto value = InterruptTiming::sample();
	assert(std::isnan(value.total_us) && std::isnan(value.logging_us));
	assert(std::isnan(value.max_us) && std::isnan(value.period_us));
}

int main()
{
	unavailable();
	InterruptTiming::initialize();
	assert(InterruptTiming::available());
	unavailable();
	CycleCounter::state().cycles = 1234;
	assert(InterruptTiming::now() == 1234);
	InterruptTiming::set_logging_cycles(32);
	InterruptTiming::record(0xfffffff0U, 0x30U);
	auto value = InterruptTiming::sample();
	assert(value.total_us == 1.0f && value.logging_us == 0.5f && value.max_us == 1.0f);
	assert(std::isnan(value.period_us));
	InterruptTiming::set_logging_cycles(64);
	assert(InterruptTiming::sample().logging_us == 0.5f); // A sample always describes the completed ISR.
	InterruptTiming::record(0xf9f0U, 0xf9f0U + 6400U);
	value = InterruptTiming::sample();
	assert(value.total_us == 100.0f && value.logging_us == 1.0f);
	assert(value.period_us == 1000.0f && value.max_us == 100.0f);
	InterruptTiming::record(0xf9f0U + 64000U, 0xf9f0U + 64000U + 3200U);
	assert(InterruptTiming::sample().total_us == 50.0f);
	assert(InterruptTiming::sample().max_us == 100.0f);
	test_primask = 1;
	InterruptTiming::reset();
	assert(test_primask == 1);
	unavailable();
	test_primask = 0;
	InterruptTiming::set_logging_cycles(16);
	InterruptTiming::record(100, 132);
	assert(InterruptTiming::sample().max_us == 0.5f);
	assert(std::isnan(InterruptTiming::sample().period_us));
	// The same microsecond durations must survive the 50 -> 100 MHz core change.
	const uint32_t test_frequencies[] = {50000000U, 100000000U};
	for (const uint32_t hz : test_frequencies) {
		CycleCounter::state().hz = hz;
		InterruptTiming::initialize();
		unavailable();
		InterruptTiming::set_logging_cycles(hz / 50000U);
		InterruptTiming::record(0U, hz / 4000U);
		InterruptTiming::record(hz / 1000U, hz / 1000U + hz / 4000U);
		value = InterruptTiming::sample();
		assert(fabsf(value.total_us - 250.0f) < 0.01f);
		assert(fabsf(value.logging_us - 20.0f) < 0.01f);
		assert(fabsf(value.period_us - 1000.0f) < 0.01f);
		assert(fabsf(value.max_us - 250.0f) < 0.01f);
	}
	CycleCounter::state().supported = false;
	InterruptTiming::initialize();
	assert(!InterruptTiming::available() && InterruptTiming::now() == 0);
	InterruptTiming::record(0, 10);
	unavailable();
	CycleCounter::state().supported = true;
	CycleCounter::state().hz = 0;
	InterruptTiming::initialize();
	assert(!InterruptTiming::available());
	unavailable();
	assert(test_primask == 0);
}
