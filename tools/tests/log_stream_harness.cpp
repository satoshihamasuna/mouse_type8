#include "Module/Inc/log_stream.h"
#include "Module/Inc/log_buffer.h"
#include "Module/Inc/remote_explore.h"
#include "Peripheral/Inc/communicate.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

uint32_t test_primask;
uint32_t tick;
DMA_HandleTypeDef test_dma = {};
UART_HandleTypeDef huart1 = {USART1, HAL_UART_STATE_READY, HAL_UART_STATE_READY, &test_dma};
extern volatile uint8_t rx_data;
static std::vector<uint8_t> transmitted;
static int rx_stops;
static bool fail_start;
static const uint8_t *dma_bytes;
static std::vector<uint8_t> dma_snapshot;
static std::vector<uint16_t> transfers;

uint32_t HAL_GetTick() { return tick; }
int HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return 0; }
int HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t) { return 0; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *) { ++rx_stops; return 0; }
int HAL_UART_DMAStop(UART_HandleTypeDef *) { ++rx_stops; return 0; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size)
{
	assert(uart->gState == HAL_UART_STATE_READY);
	if(fail_start) { fail_start = false; return HAL_ERROR; }
	uart->gState = HAL_UART_STATE_BUSY_TX;
	dma_bytes = bytes;
	dma_snapshot.assign(bytes, bytes + size);
	transfers.push_back(size);
	return HAL_OK;
}

static void complete_one()
{
	assert(huart1.gState == HAL_UART_STATE_BUSY_TX);
	// Emulate DMA reading later; producers must not change the active region.
	assert(memcmp(dma_bytes, dma_snapshot.data(), dma_snapshot.size()) == 0);
	transmitted.insert(transmitted.end(), dma_bytes, dma_bytes + dma_snapshot.size());
	huart1.gState = HAL_UART_STATE_READY;
	HAL_UART_TxCpltCallback(&huart1);
}

static void drain()
{
	while(huart1.gState != HAL_UART_STATE_READY) complete_one();
	assert(rx_stops == 0);
	assert(test_dma.half_interrupt_disables == transfers.size());
}

static void receive(uint8_t byte)
{
	rx_data = byte;
	HAL_UART_RxCpltCallback(&huart1);
}

static void request(const char *hex)
{
	assert(strlen(hex) == 16);
	for(int i = 0; i < 8; ++i)
	{
		char pair[] = {hex[2 * i], hex[2 * i + 1], 0};
		receive(static_cast<uint8_t>(strtoul(pair, nullptr, 16)));
	}
}

int main(int argc, char **argv)
{
#ifdef _WIN32
	_setmode(_fileno(stdout), _O_BINARY);
#endif
	assert(argc >= 2);
	Communicate_Initialize();
	Communicate_RxStartContinuous();
	uint8_t byte;
	if(strcmp(argv[1], "queue") == 0)
	{
		uint8_t bytes[128];
		memset(bytes, 0x7b, sizeof(bytes));
		test_primask = 1;
		assert(Communicate_TxTryPushBuffer(bytes, sizeof(bytes)) == 1);
		assert(test_primask == 1);
		assert(Communicate_TxTryPushBuffer(bytes, 1) == 0);
		assert(test_primask == 1);
		drain();
		assert(transmitted.size() == 128);
		test_primask = 0;
		assert(Communicate_TxTryPushBuffer(bytes, 2) == 1);
		assert(test_primask == 0);
		drain();
		assert(transmitted.size() == 130);
		return 0;
	}
	if(strcmp(argv[1], "dma-wrap") == 0)
	{
		uint8_t a[90], b[30], c[80];
		memset(a, 1, sizeof(a)); memset(b, 2, sizeof(b)); memset(c, 3, sizeof(c));
		assert(Communicate_TxTryPushBuffer(a, sizeof(a)));
		assert(Communicate_TxTryPushBuffer(b, sizeof(b)));
		complete_one();
		assert(Communicate_TxTryPushBuffer(c, sizeof(c)));
		drain();
		assert((transfers == std::vector<uint16_t>{90, 30, 8, 72}));
		assert(transmitted.size() == 200);
		for(size_t i = 0; i < transmitted.size(); ++i)
			assert(transmitted[i] == (i < 90 ? 1 : (i < 120 ? 2 : 3)));
		return 0;
	}
	if(strcmp(argv[1], "dma-retry") == 0)
	{
		uint8_t bytes[118] = {};
		fail_start = true;
		assert(Communicate_TxTryPushBuffer(bytes, sizeof(bytes)));
		assert(transfers.empty());
		Communicate_Service();
		drain();
		assert(transmitted.size() == sizeof(bytes));
		assert((transfers == std::vector<uint16_t>{118}));
		return 0;
	}
	if(strcmp(argv[1], "log-ring") == 0)
	{
		constexpr uint16_t fields = LogStream::FIELD_COUNT;
		LogBuffer<400, fields> buffer;
		half_float sample[fields], read[fields];
		assert(buffer.size() == 0 && !buffer.copy(0, read));
		for(uint16_t i = 0; i < 1234; ++i)
		{
			for(uint16_t j = 0; j < fields; ++j) sample[j].bitPattern = static_cast<uint16_t>(i * fields + j);
			buffer.push(sample);
		}
		assert(buffer.size() == 400);
		for(uint16_t i = 0; i < 400; ++i)
		{
			assert(buffer.copy(i, read));
			for(uint16_t j = 0; j < fields; ++j)
				assert(read[j].bitPattern == static_cast<uint16_t>((834 + i) * fields + j));
		}
		assert(!buffer.copy(400, read));
		buffer.reset();
		assert(buffer.size() == 0 && !buffer.copy(0, read));
		buffer.push(sample);
		assert(buffer.size() == 1 && buffer.copy(0, read));
		return 0;
	}
	if(strcmp(argv[1], "rx") == 0)
	{
		const uint8_t binary[] = {'s', 'h', 'e', 'l', 'l', '\r', 0, 0x21, 0, 'L', 'x', 0};
		for(uint8_t value : binary) receive(value);
		tick = 101; // A trailing NUL must eventually reach a binary upload reader.
		for(uint8_t value : binary)
		{
			assert(Communicate_RxTryPopData(&byte));
			assert(byte == value);
		}
		assert(!Communicate_RxTryPopData(&byte));
		return 0;
	}
	assert(argc >= 3);
	tick = strcmp(argv[1], "wrap") == 0 ? 0xfffffff0U : 100;
	if(strcmp(argv[1], "fullrx") == 0)
	{
		for(int i = 0; i < 128; ++i) receive('x');
		request(argv[2]);
		assert(LogStream::sample_due(tick));
		return 0;
	}
	request(argv[2]);
	if(strcmp(argv[1], "explore-guard") == 0)
	{
		assert(!RemoteExplore::begin(tick, 1000));
		request(argv[4]);
		assert(RemoteExplore::begin(tick, 1000));
		assert(!RemoteExplore::should_stop(tick + 999));
		assert(RemoteExplore::should_stop(tick + 1000));
		assert(RemoteExplore::stop_reason() == RemoteExplore::TIME_LIMIT);
		RemoteExplore::finish();
		request(argv[4]);
		assert(RemoteExplore::begin(tick, 10000));
		request(argv[5]);
		RemoteExplore::arm(); // A running request cannot clear its cancellation.
		assert(RemoteExplore::should_stop(tick));
		assert(RemoteExplore::stop_reason() == RemoteExplore::USER_STOP);
		RemoteExplore::finish();
		assert(!RemoteExplore::begin(tick, 1000));
		request(argv[4]);
		assert(RemoteExplore::begin(tick, 10000));
		assert(RemoteExplore::should_stop(tick + LogStream::LEASE_MS));
		assert(RemoteExplore::stop_reason() == RemoteExplore::LINK_TIMEOUT);
		RemoteExplore::finish();
		request(argv[4]); request(argv[5]);
		assert(!RemoteExplore::begin(tick, 1000)); // Stop before start is retained.
		return 0;
	}
	if(strcmp(argv[1], "invalid") == 0)
	{
		assert(!LogStream::sample_due(tick));
		return 0;
	}
	assert(!Communicate_RxTryPopData(&byte));
	assert(LogStream::sample_due(tick));
	half_float sample[LogStream::FIELD_COUNT];
	for(uint16_t i = 0; i < LogStream::FIELD_COUNT; ++i) sample[i].bitPattern = 0x3c00 + i;
	LogStream::enqueue_sample(sample, tick);
	assert(transfers.empty()); // ADC enqueue never starts DMA or computes a CRC.
	Communicate_Service();
	assert((transfers == std::vector<uint16_t>{LogStream::FRAME_SIZE}));
	if(strcmp(argv[1], "keepalive") == 0)
	{
		drain();
		tick += 2500;
		request(argv[2]);
		assert(LogStream::sample_due(tick));
		LogStream::enqueue_sample(sample, tick);
		Communicate_Service();
		drain();
		assert(transmitted[LogStream::FRAME_SIZE + 4] == 1);
		assert(LogStream::sample_due(tick + 1000));
		assert(!LogStream::sample_due(tick + LogStream::LEASE_MS));
		return 0;
	}
	if(strcmp(argv[1], "stream-stop") == 0)
	{
		tick += 20;
		assert(LogStream::sample_due(tick));
		LogStream::enqueue_sample(sample, tick);
		Communicate_Service(); // Prepare, but TX is still full.
		request(argv[3]);
		Communicate_Service();
		drain();
		Communicate_Service();
		assert(transmitted.size() == LogStream::FRAME_SIZE);
		return 0;
	}
	assert(!LogStream::sample_due(tick + 19));
	// Keep DMA blocked. The prepared frame + 8 ring slots hold the burst.
	for(uint32_t i = 1; i <= 10; ++i)
	{
		assert(LogStream::sample_due(tick + 20 * i));
		LogStream::enqueue_sample(sample, tick + 20 * i);
		LogStream::service(tick + 20 * i);
	}
	drain();
	for(uint8_t i = 0; i < 9; ++i) { Communicate_Service(); drain(); }
	assert(transmitted.size() == 10 * LogStream::FRAME_SIZE);
	assert(LogStream::sample_due(tick + 220));
	LogStream::enqueue_sample(sample, tick + 220);
	Communicate_Service(); drain();
	assert(transmitted[10 * LogStream::FRAME_SIZE + 4] == 11); // Attempt 10 was dropped.
	assert(!LogStream::sample_due(tick + LogStream::LEASE_MS));
	tick += 5000;
	request(argv[2]);
	assert(LogStream::sample_due(tick));
	LogStream::enqueue_sample(sample, tick);
	Communicate_Service(); drain();
	assert(argc >= 4);
	request(argv[3]);
	assert(!LogStream::sample_due(tick + 1000));
	assert(transfers.size() == 12);
	for(uint16_t length : transfers) assert(length == LogStream::FRAME_SIZE);
	fwrite(transmitted.data(), 1, transmitted.size(), stdout);
	return 0;
}
