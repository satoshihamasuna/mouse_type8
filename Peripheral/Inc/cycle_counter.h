#ifndef PERIPHERAL_INC_CYCLE_COUNTER_H_
#define PERIPHERAL_INC_CYCLE_COUNTER_H_

#include "Core/Inc/main.h"

namespace CycleCounter
{
inline bool enable()
{
	DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
	if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U) return false;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	__DSB();
	__ISB();
	const uint32_t before = DWT->CYCCNT;
	for(int i = 0; i < 8; ++i) __NOP();
	return DWT->CYCCNT != before;
}
inline uint32_t read() { return DWT->CYCCNT; }
inline uint32_t frequency() { return SystemCoreClock; }
}

#endif
