#ifndef TEST_LOG_STREAM_MAIN_H
#define TEST_LOG_STREAM_MAIN_H
#include <stdint.h>
extern uint32_t test_primask;
inline uint32_t __get_PRIMASK() { return test_primask; }
inline void __disable_irq() { test_primask = 1; }
inline void __enable_irq() { test_primask = 0; }
inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
uint32_t HAL_GetTick();
#endif
