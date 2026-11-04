#include "Module/Inc/log_stream.h"
#include "Peripheral/Inc/communicate.h"
#include "Module/Inc/remote_explore.h"
#include <string.h>

namespace
{
uint8_t control[8];
uint8_t control_size;
uint32_t control_time;
volatile uint16_t period_ms;
volatile uint32_t last_request;
volatile uint32_t generation;
uint32_t next_sample;
uint32_t sequence;
struct PendingFrame
{
	uint8_t bytes[LogStream::FRAME_SIZE];
	uint32_t generation;
};
PendingFrame pending[LogStream::QUEUE_SIZE];
uint8_t pending_read, pending_write;
volatile uint8_t pending_count;
uint8_t prepared[LogStream::FRAME_SIZE];
uint32_t prepared_generation;
bool prepared_ready;

uint16_t crc16(const uint8_t *bytes, uint16_t length)
{
	uint16_t crc = 0xffffU;
	for(uint16_t i = 0; i < length; ++i)
	{
		crc ^= static_cast<uint16_t>(bytes[i]) << 8;
		for(uint8_t bit = 0; bit < 8; ++bit)
		{
			crc = (crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1;
		}
	}
	return crc;
}

void put16(uint8_t *out, uint16_t value)
{
	out[0] = static_cast<uint8_t>(value);
	out[1] = static_cast<uint8_t>(value >> 8);
}

void put32(uint8_t *out, uint32_t value)
{
	put16(out, static_cast<uint16_t>(value));
	put16(out + 2, static_cast<uint16_t>(value >> 16));
}
}

void LogStream::flush_rx(uint32_t now_ms, void (*forward)(uint8_t))
{
	if(control_size != 0 && now_ms - control_time > 100U)
	{
		for(uint8_t i = 0; i < control_size; ++i) forward(control[i]);
		control_size = 0;
	}
}

void LogStream::receive_byte(uint8_t byte, uint32_t now_ms, void (*forward)(uint8_t))
{
	flush_rx(now_ms, forward);
	if(control_size == 0 && byte != 0)
	{
		forward(byte);
		return;
	}
	control_time = now_ms;
	control[control_size++] = byte;
	if((control_size == 2 && byte != 'L') || (control_size == 3 && byte != '8'))
	{
		for(uint8_t i = 0; i < control_size; ++i) forward(control[i]);
		control_size = 0;
		return;
	}
	if(control_size < sizeof(control)) return;
	control_size = 0;
	const uint16_t requested = control[4] | (static_cast<uint16_t>(control[5]) << 8);
	const uint16_t checksum = control[6] | (static_cast<uint16_t>(control[7]) << 8);
	if(control[0] != 0 || control[1] != 'L' || control[2] != '8' ||
	   crc16(control, 6) != checksum)
	{
		for(uint8_t i = 0; i < sizeof(control); ++i) forward(control[i]);
		return;
	}
	if(control[3] == 0)
	{
		period_ms = 0;
		++generation;
	}
	else if(control[3] == 2) RemoteExplore::request_stop();
	else if(control[3] == 3) RemoteExplore::arm();
	else if(control[3] == 1 && requested >= MIN_PERIOD_MS && requested <= MAX_PERIOD_MS)
	{
		if(period_ms == 0 || now_ms - last_request >= LEASE_MS)
		{
			next_sample = now_ms;
			++generation;
		}
		period_ms = requested;
		last_request = now_ms;
	}
}

bool LogStream::active(uint32_t now_ms)
{
	return period_ms != 0 && now_ms - last_request < LEASE_MS;
}

bool LogStream::sample_due(uint32_t now_ms)
{
	if(period_ms == 0) return false;
	if(now_ms - last_request >= LEASE_MS)
	{
		period_ms = 0;
		++generation;
		return false;
	}
	if(static_cast<int32_t>(now_ms - next_sample) < 0) return false;
	next_sample = now_ms + period_ms;
	return true;
}

void LogStream::enqueue_sample(const half_float *sample, uint32_t now_ms)
{
	static_assert(sizeof(half_float) == 2, "Stream requires binary16 storage");
	const uint32_t index = sequence++;
	if(pending_count == QUEUE_SIZE) return;
	PendingFrame &slot = pending[pending_write];
	uint8_t *frame = slot.bytes;
	memcpy(frame, "L8S2", 4);
	put32(frame + 4, index); // Count attempts, including whole-frame drops.
	put32(frame + 8, now_ms);
	put16(frame + 12, period_ms);
	// STM32 is little endian. Copy the already-converted half values once.
	memcpy(frame + 14, sample, sizeof(half_float) * FIELD_COUNT);
	slot.generation = generation;
	pending_write = (pending_write + 1) % QUEUE_SIZE;
	++pending_count;
}

void LogStream::service(uint32_t now_ms)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if(period_ms == 0 || now_ms - last_request >= LEASE_MS)
	{
		pending_read = pending_write;
		pending_count = 0;
		prepared_ready = false;
		__set_PRIMASK(primask);
		return;
	}
	if(prepared_ready && prepared_generation != generation) prepared_ready = false;
	if(!prepared_ready && pending_count != 0)
	{
		memcpy(prepared, pending[pending_read].bytes, FRAME_SIZE - 2);
		prepared_generation = pending[pending_read].generation;
		pending_read = (pending_read + 1) % QUEUE_SIZE;
		--pending_count;
		__set_PRIMASK(primask);
		// CRC work is preemptible by the control ISR. Build once, even if TX is full.
		put16(prepared + FRAME_SIZE - 2, crc16(prepared, FRAME_SIZE - 2));
		prepared_ready = true;
		primask = __get_PRIMASK();
		__disable_irq();
	}
	if(prepared_ready && prepared_generation == generation && period_ms != 0 &&
	   Communicate_TxTryPushBuffer(prepared, FRAME_SIZE) != 0U)
	{
		prepared_ready = false;
	}
	__set_PRIMASK(primask);
}
