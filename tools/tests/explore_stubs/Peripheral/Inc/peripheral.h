#ifndef TEST_EXPLORE_PERIPHERAL_H_
#define TEST_EXPLORE_PERIPHERAL_H_
#include <stdint.h>
#include "Peripheral/Inc/ir_sensor_activity.h"
inline void Sensor_SetHandWaitActive(uint8_t) {}
inline uint8_t Sensor_IrSamplesReady(void) { return 1U; }
void HAL_Delay(uint32_t ms);
inline void Indicate_LED(uint8_t) {}
#endif
