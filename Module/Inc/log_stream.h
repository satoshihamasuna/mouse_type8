#ifndef MODULE_INC_LOG_STREAM_H_
#define MODULE_INC_LOG_STREAM_H_

#include "Component/Inc/half_float.h"

namespace LogStream
{
constexpr uint16_t MIN_PERIOD_MS = 20;
constexpr uint16_t MAX_PERIOD_MS = 1000;
constexpr uint32_t LEASE_MS = 3000;
constexpr uint16_t FIELD_COUNT = 55;
constexpr uint16_t FRAME_SIZE = 16 + FIELD_COUNT * 2;
constexpr uint8_t QUEUE_SIZE = 8;

// UART RX and ADC callbacks have the same preemption priority.
// NUL-prefixed control packets are consumed before the shell RX queue.
void receive_byte(uint8_t byte, uint32_t now_ms, void (*forward)(uint8_t));
void flush_rx(uint32_t now_ms, void (*forward)(uint8_t));
bool sample_due(uint32_t now_ms);
bool active(uint32_t now_ms);
void enqueue_sample(const half_float *sample, uint32_t now_ms);
// Called from SysTick (priority 15), below the ADC/UART interrupts (priority 0).
void service(uint32_t now_ms);
}

#endif
