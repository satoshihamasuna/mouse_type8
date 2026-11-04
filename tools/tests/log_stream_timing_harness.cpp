// Host-only UART timing model. Actual stream/TX implementations are used.
// CRC/HAL CPU cost, DMA bus contention and BLE are deliberately not modeled.
#include "Module/Inc/log_stream.h"
#include "Peripheral/Inc/communicate.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

uint32_t test_primask;
uint32_t tick;
DMA_HandleTypeDef test_dma = {};
UART_HandleTypeDef huart1 = {USART1, HAL_UART_STATE_READY, HAL_UART_STATE_READY, &test_dma};
static constexpr uint64_t NEVER = std::numeric_limits<uint64_t>::max();
static uint64_t sim_now, dma_due, last_wire_end, idle_sum, idle_max;
static uint32_t baud = 460800, starts;
static bool uart_pending;
static const uint8_t *dma_pointer;
static std::vector<uint8_t> dma_original, wire;
static std::vector<uint16_t> dma_lengths;

uint32_t HAL_GetTick() { return tick; }
int HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *bytes, uint16_t size, uint32_t) { memset(bytes, 0, size); return HAL_OK; }
int HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *) { assert(false); return HAL_ERROR; }
int HAL_UART_DMAStop(UART_HandleTypeDef *) { assert(false); return HAL_ERROR; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size)
{
    assert(test_primask == 1 && uart->gState == HAL_UART_STATE_READY && dma_due == NEVER);
    if(starts != 0)
    {
        assert(sim_now >= last_wire_end);
        const uint64_t idle = sim_now - last_wire_end;
        idle_sum += idle;
        idle_max = std::max(idle_max, idle);
    }
    ++starts;
    dma_lengths.push_back(size);
    dma_pointer = bytes;
    dma_original.assign(bytes, bytes + size);
    // Ten bits per UART byte, exact ceiling at the requested transmitter baud.
    dma_due = sim_now + (static_cast<uint64_t>(size) * 10000000000ULL + baud - 1) / baud;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

namespace RemoteExplore { void request_stop() { assert(false); } void arm() { assert(false); } }
#ifdef LEGACY_TX128
#include "communicate_legacy.cpp"
#else
#include "../../Peripheral/Src/communicate.cpp"
#endif
#include "../../Module/Src/log_stream.cpp"

static void send_request(uint16_t period)
{
    uint8_t request[8] = {0, 'L', '8', 1, static_cast<uint8_t>(period), static_cast<uint8_t>(period >> 8), 0, 0};
    const uint16_t checksum = crc16(request, 6);
    request[6] = checksum;
    request[7] = checksum >> 8;
    for(uint8_t byte : request)
    {
        rx_data = byte;
        HAL_UART_RxCpltCallback(&huart1);
    }
}

static uint32_t u32(const uint8_t *in)
{
    return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
        (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

static void run(uint32_t phase_us, uint32_t adc_busy_us, uint32_t period, uint32_t seconds)
{
    // Globals normally begin zeroed once per boot; reset them between independent phases.
    control_size = 0; control_time = 0; period_ms = 0; last_request = 0; generation = 0;
    next_sample = sequence = 0; pending_read = pending_write = pending_count = 0;
    prepared_generation = 0; prepared_ready = false;
    test_primask = tick = starts = 0;
    test_dma.half_interrupt_disables = 0;
    sim_now = last_wire_end = idle_sum = idle_max = 0;
    dma_due = NEVER; uart_pending = false;
    wire.clear(); dma_original.clear(); dma_lengths.clear();
    huart1.gState = huart1.RxState = HAL_UART_STATE_READY;
    Communicate_Initialize();
    send_request(period);

    const uint64_t end = static_cast<uint64_t>(seconds) * 1000000000ULL;
    uint64_t adc_due = static_cast<uint64_t>(phase_us) * 1000;
    uint64_t adc_end = NEVER, tick_due = 1000000;
    bool tick_pending = false, adc_active = false;
    uint32_t dropped = 0, max_pending = 0, max_tx_used = 0, max_buffered = 0;
    uint32_t attempted_at_end = 0, wire_at_end = 0, systick_coalesced = 0;
    uint64_t first_drop = NEVER;
    bool recorded_end = false;
    half_float sample[LogStream::FIELD_COUNT];
    for(uint16_t i = 0; i < LogStream::FIELD_COUNT; ++i) sample[i].bitPattern = 0x3c00 + i;

    // Stop ADC generation at end, but drain all accepted frames without cancelling the stream.
    while(!recorded_end || pending_count || prepared_ready || tx_used || adc_active)
    {
        const uint64_t next = std::min(std::min(adc_due, adc_end), std::min(tick_due, dma_due));
        assert(next != NEVER && next >= sim_now && next < end + 1000000000ULL);
        sim_now = next;
        if(!recorded_end && sim_now >= end)
        {
            attempted_at_end = sequence;
            wire_at_end = wire.size();
            recorded_end = true;
            adc_due = NEVER;
        }
        if(dma_due == sim_now)
        {
            // DMA reads later: no producer may overwrite the reserved active region.
            assert(dma_original.size() != 0);
            assert(memcmp(dma_pointer, dma_original.data(), dma_original.size()) == 0);
            wire.insert(wire.end(), dma_pointer, dma_pointer + dma_original.size());
            dma_due = NEVER;
            last_wire_end = sim_now;
            uart_pending = true;
        }
        if(tick_due == sim_now)
        {
            if(tick_pending) ++systick_coalesced;
            tick_pending = true;
            tick_due += 1000000;
        }
        if(adc_due == sim_now)
        {
            assert(!adc_active);
            adc_active = true;
            adc_end = sim_now + static_cast<uint64_t>(adc_busy_us) * 1000;
            adc_due += 1018000;
            if(adc_due >= end) adc_due = NEVER;
        }
        if(adc_end == sim_now)
        {
            // Logging is at the end of ADC callback, before a pending low-priority SysTick.
            if(sim_now < end && LogStream::sample_due(tick))
            {
                if(pending_count == LogStream::QUEUE_SIZE)
                {
                    ++dropped;
                    first_drop = std::min(first_drop, sim_now);
                }
                LogStream::enqueue_sample(sample, tick);
            }
            adc_active = false;
            adc_end = NEVER;
        }
        if(!adc_active)
        {
            // ADC and UART have priority 0, SysTick priority 15. UART TC cannot preempt ADC.
            if(uart_pending)
            {
                uart_pending = false;
                huart1.gState = HAL_UART_STATE_READY;
                HAL_UART_TxCpltCallback(&huart1);
            }
            if(tick_pending)
            {
                tick_pending = false;
                ++tick;
                // Synthetic zero-cost keepalive only; no UART traffic or stream reset.
                if(tick % 1000 == 0) send_request(period);
                Communicate_Service();
            }
        }
        max_pending = std::max(max_pending, static_cast<uint32_t>(pending_count));
        max_tx_used = std::max(max_tx_used, static_cast<uint32_t>(tx_used));
        max_buffered = std::max(max_buffered, static_cast<uint32_t>(pending_count) +
            (prepared_ready ? 1U : 0U) + (static_cast<uint32_t>(tx_used) + 125U) / 126U);
    }
    assert(test_primask == 0 && !uart_pending && dma_due == NEVER);
    assert(test_dma.half_interrupt_disables == starts);
    assert(wire.size() % LogStream::FRAME_SIZE == 0);
    uint32_t next_sequence = 0, gaps = 0, crc_errors = 0, partial_transfers = 0;
    for(size_t offset = 0; offset < wire.size(); offset += LogStream::FRAME_SIZE)
    {
        const uint8_t *frame = wire.data() + offset;
        assert(memcmp(frame, "L8S2", 4) == 0);
        const uint32_t index = u32(frame + 4);
        assert(index >= next_sequence);
        gaps += index - next_sequence;
        next_sequence = index + 1;
        const uint16_t received_crc = frame[124] | (static_cast<uint16_t>(frame[125]) << 8);
        if(crc16(frame, 124) != received_crc) ++crc_errors;
    }
    gaps += sequence - next_sequence;
    assert(gaps == dropped && crc_errors == 0);
    for(uint16_t size : dma_lengths) if(size % LogStream::FRAME_SIZE) ++partial_transfers;
    printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.3f,%u,%u,%u,%u\n",
        static_cast<unsigned>(TX_BUFFER_SIZE),
        phase_us, adc_busy_us, period, seconds, attempted_at_end,
        static_cast<unsigned>(wire.size() / LogStream::FRAME_SIZE), dropped, gaps, crc_errors,
        max_pending, max_tx_used, first_drop == NEVER ? -1.0 : first_drop / 1000000.0,
        idle_max / 1000.0, starts > 1 ? idle_sum / 1000.0 / (starts - 1) : 0.0,
        starts, partial_transfers, wire_at_end, systick_coalesced);
}

int main(int argc, char **argv)
{
    assert(argc == 6);
    const uint32_t seconds = std::strtoul(argv[1], nullptr, 10);
    const uint32_t busy = std::strtoul(argv[2], nullptr, 10);
    const uint32_t period = std::strtoul(argv[3], nullptr, 10);
    const uint32_t step = std::strtoul(argv[4], nullptr, 10);
    baud = std::strtoul(argv[5], nullptr, 10);
    assert(seconds > 0 && busy < 1018 && period >= 3 && step > 0 && baud > 0);
    puts("tx_bytes,phase_us,adc_busy_us,period_ms,duration_s,attempted,received_after_drain,dropped,gaps,crc_errors,max_pending,max_tx_used,first_drop_ms,max_dma_gap_us,mean_dma_gap_us,dma_starts,split_transfers,wire_bytes_by_end,systick_coalesced");
    for(uint32_t phase = 0; phase < 1018; phase += step) run(phase, busy, period, seconds);
}
