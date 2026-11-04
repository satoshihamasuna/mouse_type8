#ifndef MODULE_INC_INTERRUPT_TIMING_H_
#define MODULE_INC_INTERRUPT_TIMING_H_

#include <stdint.h>

namespace InterruptTiming
{
struct Sample
{
	float total_us;
	float logging_us;
	float max_us;
	float period_us;
};

void initialize();
bool available();
uint32_t now();
void set_logging_cycles(uint32_t elapsed);
void record(uint32_t entered, uint32_t finished);
void reset();
// Called by the ADC ISR before record(): all fields refer to its previous invocation.
Sample sample();
}

#endif
