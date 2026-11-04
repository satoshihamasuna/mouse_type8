"""停止中のマウスでBLEログ周期を段階測定する。走行・探索許可は送信しない。"""
import argparse
import asyncio
import binascii
from datetime import datetime
import json
import math
from pathlib import Path
import re
import statistics
import struct
import sys
import time

import mouse_ble_gateway as transport
import mouse_ble_log as log
from mouse_ble_shell import send_command


RAW_MAGIC = b'M8N1'
RAW_HEADER = struct.Struct('<dI')  # session-relative monotonic seconds, payload bytes
STATUS_NAMES = ('dropped_uart_bytes', 'uart_errors', 'queue_busy', 'ble_errors')
DIAGNOSTIC_ERROR_COUNTERS = (
    'overrun_count', 'parity_count', 'framing_count', 'break_count',
    'collision_count', 'noise_count', 'other_reason_count',
    'rx_restart_failures', 'rx_buffer_alloc_failures', 'rx_buffer_rsp_failures',
)
CAPABILITY = re.compile(r'EXPLORE_CAP\b[^\r\n]*\bmouse:([AB])\b[^\r\n]*\bstate:(\w+)')


class BenchmarkError(RuntimeError):
    pass


def describe(values):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return {'count': 0, 'nonfinite': len(values)}
    def percentile(fraction):
        position = (len(finite) - 1) * fraction
        low = int(position)
        return finite[low] + (finite[min(low + 1, len(finite) - 1)] - finite[low]) * (position - low)
    return {'count': len(finite), 'nonfinite': len(values) - len(finite),
            'min': finite[0], 'mean': statistics.fmean(finite), 'max': finite[-1],
            'median': percentile(0.5), 'p95': percentile(0.95), 'p99': percentile(0.99)}


def status_delta(before, after):
    return dict(zip(STATUS_NAMES, ((end - start) & 0xffffffff for start, end in zip(before, after))))


def require_healthy(before, after):
    delta = status_delta(before, after)
    if any(delta.values()):
        raise BenchmarkError('Bridge counters increased: {}'.format(delta))


def diagnostics_delta(before, after):
    return {name: (after[name] - before[name]) & 0xffffffff
            for name in transport.DIAGNOSTICS_COUNTER_FIELDS}


def require_diagnostics_healthy(before, after):
    delta = diagnostics_delta(before, after)
    errors = {name: delta[name] for name in DIAGNOSTIC_ERROR_COUNTERS if delta[name]}
    if errors:
        raise BenchmarkError('Bridge diagnostic errors increased: {}'.format(errors))


def link_diagnostics_delta(before, after):
    return {name: (after[name] - before[name]) & 0xffffffff
            for name in transport.LINK_DIAGNOSTICS_COUNTER_FIELDS}


def require_link_healthy(before, after):
    delta = link_diagnostics_delta(before, after)
    if delta['notify_failures']:
        raise BenchmarkError('Bridge notification failures increased: {}'.format(delta['notify_failures']))


def link_connection_metrics(snapshot):
    """Expose actual peripheral observations, preserving unknown negotiated fields."""
    result = {'connection_interval_ms': None, 'phy': None, 'data_length': None,
              'link_metrics_source': 'unknown'}
    if snapshot is None:
        return result
    result['link_metrics_source'] = 'HY0020 link diagnostics'
    if snapshot['interval_us']:
        result['connection_interval_ms'] = snapshot['interval_us'] / 1000
    if snapshot['tx_phy'] or snapshot['rx_phy']:
        result['phy'] = {'tx': snapshot['tx_phy'] or None, 'rx': snapshot['rx_phy'] or None}
    length_fields = {'tx_octets': 'tx_max_len', 'tx_time_us': 'tx_max_time_us',
                     'rx_octets': 'rx_max_len', 'rx_time_us': 'rx_max_time_us'}
    if any(snapshot[name] for name in length_fields.values()):
        result['data_length'] = {name: snapshot[field] or None for name, field in length_fields.items()}
    return result


def require_idle(text, mouse):
    matches = list(CAPABILITY.finditer(text))
    if not matches or matches[-1].groups() != (mouse, 'idle'):
        raise BenchmarkError('Expected mouse:{} state:idle in explore show response'.format(mouse))


def parse_periods(value):
    try:
        result = tuple(int(item.strip()) for item in value.split(','))
    except ValueError as error:
        raise argparse.ArgumentTypeError('periods must be comma-separated integers') from error
    if not result or any(not 3 <= item <= 1000 for item in result):
        raise argparse.ArgumentTypeError('each period must be 3..1000 ms')
    return result


class SegmentCapture:
    """Filter old-period frames before recording, using the shared wire decoder/CSV writer."""
    def __init__(self, file, period_ms):
        self.period_ms = period_ms
        self.decoder = log.StreamDecoder()
        self.csv = log.CsvCapture(file)
        self.stale_frames = 0
        self.first_rx = self.last_rx = self.first_device = None
        self.previous_device = None
        self.intervals = []
        self.relative_lag = []
        self.device_elapsed = []
        self.irq = {name: [] for name in ('total_us', 'logging_us', 'period_us')}
        self.irq_max_start = self.irq_max_end = None

    def feed(self, data, received_at):
        samples = self.decoder.feed(data)
        if self.decoder.crc_errors:
            raise BenchmarkError('CRC error in the received log stream')
        accepted = 0
        for sample in samples:
            if sample.period_ms != self.period_ms:
                if self.csv.rows:
                    raise BenchmarkError('Log period changed from {} to {} ms'.format(
                        self.period_ms, sample.period_ms))
                self.stale_frames += 1
                continue
            if len(sample.values) != 55:
                raise BenchmarkError('Expected 55 fields including IRQ timing')
            # CsvCapture accepts wire bytes. Rebuild a validated sample for its shared
            # column ordering, wrap checks and loss accounting; original bytes stay in raw.
            payload = struct.pack('<4sIIH55e', log.MAGIC, sample.sequence, sample.time_ms,
                                  sample.period_ms, *sample.values)
            self.csv.feed(payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff)))
            if self.first_rx is None:
                self.first_rx, self.first_device = received_at, sample.time_ms
                self.irq_max_start = sample.values[53] if math.isfinite(sample.values[53]) else None
            if self.previous_device is not None:
                self.intervals.append((sample.time_ms - self.previous_device) & 0xffffffff)
            elapsed = (sample.time_ms - self.first_device) & 0xffffffff
            self.device_elapsed.append(elapsed / 1000)
            self.relative_lag.append((received_at - self.first_rx) * 1000 - elapsed)
            self.previous_device, self.last_rx = sample.time_ms, received_at
            for name, index in (('total_us', 51), ('logging_us', 52), ('period_us', 54)):
                self.irq[name].append(sample.values[index])
            if math.isfinite(sample.values[53]):
                self.irq_max_end = sample.values[53]
            accepted += 1
            if self.csv.lost:
                raise BenchmarkError('Missing {} log frame(s)'.format(self.csv.lost))
        return accepted

    def report(self):
        interval = describe(self.intervals)
        span = self.device_elapsed[-1] if self.device_elapsed else 0
        slope = None
        if len(self.device_elapsed) > 1:
            mean_x = statistics.fmean(self.device_elapsed)
            mean_y = statistics.fmean(self.relative_lag)
            denominator = sum((x - mean_x) ** 2 for x in self.device_elapsed)
            if denominator:
                slope = sum((x - mean_x) * (y - mean_y)
                            for x, y in zip(self.device_elapsed, self.relative_lag)) / denominator
        return {'rows': self.csv.rows, 'lost_frames': self.csv.lost,
                'crc_errors': self.decoder.crc_errors, 'stale_frames_ignored': self.stale_frames,
                'device_span_s': span, 'interval_ms': interval,
                'effective_hz': (self.csv.rows - 1) / span if span else None,
                'cadence_within_5_percent': bool(self.intervals) and
                    interval['mean'] <= self.period_ms * 1.05 and min(self.intervals) >= self.period_ms,
                'relative_receive_lag_ms': describe(self.relative_lag),
                'relative_lag_slope_ms_per_s': slope,
                'relative_lag_note': 'Includes PC/device clock drift and transport buffering; not absolute latency.',
                'irq': {name: describe(values) for name, values in self.irq.items()},
                'irq_cumulative_max_start_us': self.irq_max_start,
                'irq_cumulative_max_end_us': self.irq_max_end}


async def stream_control(client, lock, period_ms=20, *, stop=False):
    # Only stream start/keepalive (1) and stream stop (0). No motion/arm packets.
    await asyncio.wait_for(transport.write_bytes(
        client, log.control_packet(period_ms, stop=stop), lock), 2)


async def read_status(client, lock):
    async with lock:
        return transport.decode_status(await asyncio.wait_for(client.read_gatt_char(transport.STATUS), 2))


async def read_diagnostics(client, lock):
    return await asyncio.wait_for(transport.read_diagnostics(client, lock), 2)


async def read_link_diagnostics(client, lock):
    return await asyncio.wait_for(transport.read_link_diagnostics(client, lock), 2)


async def drain(incoming, seconds=0.3):
    deadline = time.monotonic() + seconds
    discarded = 0
    while time.monotonic() < deadline:
        try:
            _, data = await asyncio.wait_for(incoming.get(), min(0.1, deadline - time.monotonic()))
            discarded += len(data)
        except asyncio.TimeoutError:
            pass
    return discarded


async def query_idle(client, lock, incoming, mouse, enter_shell):
    if enter_shell:
        await send_command(client, 'shell', lock, 0.08)
        await asyncio.sleep(0.5)
    await send_command(client, 'explore show', lock, 0.08)
    text = ''
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            _, data = await asyncio.wait_for(incoming.get(), 0.1)
        except asyncio.TimeoutError:
            continue
        text += data.decode('utf-8', errors='replace')
        complete = text.rsplit('\n', 1)[0] if '\n' in text else ''
        if CAPABILITY.search(complete):
            require_idle(complete, mouse)
            return text
        if len(text) > 16384:
            raise BenchmarkError('No idle capability response within 16 KiB')
    require_idle(text, mouse)
    return text


async def capture_segment(client, lock, incoming, state, directory, index, period, duration):
    stem = '{:02d}_{}ms'.format(index, period)
    report = {'period_ms': period, 'requested_duration_s': duration, 'passed': False,
              'csv': stem + '.csv', 'raw_notifications': stem + '.notifications.bin',
              'raw_format': {'magic': RAW_MAGIC.decode(), 'record_header': '<dI',
                             'timestamp': 'monotonic seconds since segment start'},
              'notes': ['IRQ sample fields describe the previous completed control ISR.',
                        'irq_cumulative_max is since firmware timing reset, not this segment.',
                        'Idle tests do not establish exploration-load safety.']}
    before = after = None
    diagnostics_before = diagnostics_after = diagnostics_last_observed = None
    diagnostics_supported = state.get('diagnostics_supported')
    link_before = link_after = link_last_observed = None
    link_supported = state.get('link_supported')
    error = None
    capture = None
    received_bytes = notifications = 0
    notification_sizes, notification_intervals = [], []
    previous_rx = first_rx = last_rx = None
    started = time.monotonic()
    segment_highwater = 0

    async def observe_diagnostics():
        nonlocal diagnostics_supported, diagnostics_last_observed
        try:
            current = await read_diagnostics(client, lock)
            if current is None and diagnostics_supported is True:
                raise BenchmarkError('Bridge diagnostics characteristic disappeared')
        except Exception as diagnostic_error:
            report['diagnostics_read_error'] = '{}: {}'.format(type(diagnostic_error).__name__, diagnostic_error)
            raise
        diagnostics_supported = current is not None
        state['diagnostics_supported'] = diagnostics_supported
        if current is not None:
            diagnostics_last_observed = dict(current)
        return current

    async def observe_link():
        nonlocal link_supported, link_last_observed
        try:
            current = await read_link_diagnostics(client, lock)
            if current is None and link_supported is True:
                raise BenchmarkError('Bridge link diagnostics characteristic disappeared')
        except Exception as link_error:
            report['link_read_error'] = '{}: {}'.format(type(link_error).__name__, link_error)
            raise
        link_supported = current is not None
        state['link_supported'] = link_supported
        if current is not None:
            link_last_observed = dict(current)
        return current

    try:
        await stream_control(client, lock, stop=True)
        report['pre_drain_bytes'] = await drain(incoming)
        before = await read_status(client, lock)
        diagnostics_before = await observe_diagnostics()
        link_before = await observe_link()
        with (directory / report['csv']).open('x', newline='', encoding='utf-8') as csv_file, \
                (directory / report['raw_notifications']).open('xb') as raw:
            capture = SegmentCapture(csv_file, period)
            raw.write(RAW_MAGIC)
            started = last_valid = time.monotonic()
            await stream_control(client, lock, period)
            next_request, next_health = time.monotonic() + 0.5, time.monotonic() + 1
            while time.monotonic() - started < duration:
                if state['failure']:
                    raise BenchmarkError(state['failure'])
                now = time.monotonic()
                if now >= next_request:
                    await stream_control(client, lock, period)
                    next_request = time.monotonic() + 0.5
                if now >= next_health:
                    after = await read_status(client, lock)
                    if diagnostics_supported:
                        diagnostics_after = await observe_diagnostics()
                        require_diagnostics_healthy(diagnostics_before, diagnostics_after)
                    if link_supported:
                        link_after = await observe_link()
                        require_link_healthy(link_before, link_after)
                    require_healthy(before, after)
                    next_health = time.monotonic() + 1
                segment_highwater = max(segment_highwater, incoming.qsize())
                try:
                    received_at, data = await asyncio.wait_for(incoming.get(), 0.1)
                except asyncio.TimeoutError:
                    if time.monotonic() - last_valid >= 5:
                        raise BenchmarkError('No matching valid frame for 5 seconds')
                    continue
                raw.write(RAW_HEADER.pack(received_at - started, len(data)))
                raw.write(data)
                notifications += 1
                received_bytes += len(data)
                notification_sizes.append(len(data))
                if previous_rx is not None:
                    notification_intervals.append((received_at - previous_rx) * 1000)
                first_rx = received_at if first_rx is None else first_rx
                last_rx = previous_rx = received_at
                if capture.feed(data, received_at):
                    last_valid = received_at
                if time.monotonic() - last_valid >= 5:
                    raise BenchmarkError('No matching valid frame for 5 seconds')
            if capture.csv.rows < 2:
                raise BenchmarkError('Fewer than two matching log samples')
            report['sample'] = capture.report()
            if not report['sample']['cadence_within_5_percent']:
                raise BenchmarkError('Effective sample cadence missed the requested period by over 5 percent')
            after = await read_status(client, lock)
            if diagnostics_supported:
                diagnostics_after = await observe_diagnostics()
                require_diagnostics_healthy(diagnostics_before, diagnostics_after)
            if link_supported:
                link_after = await observe_link()
                require_link_healthy(link_before, link_after)
            require_healthy(before, after)
            report['passed'] = True
    except (Exception, asyncio.CancelledError) as caught:
        error = caught
        report['error'] = '{}: {}'.format(type(caught).__name__, caught)
    finally:
        if capture is not None:
            report['sample'] = capture.report()
        report['capture_elapsed_s'] = time.monotonic() - started
        final_status_read = False
        final_diagnostics_read = False
        final_link_read = False
        try:
            await asyncio.wait_for(stream_control(client, lock, stop=True), 2)
            report['post_drain_bytes'] = await drain(incoming)
            after = await asyncio.wait_for(read_status(client, lock), 2)
            final_status_read = True
            if diagnostics_supported:
                diagnostics_after = await observe_diagnostics()
                final_diagnostics_read = True
            if link_supported:
                link_after = await observe_link()
                final_link_read = True
            if before is not None:
                require_healthy(before, after)
            if final_diagnostics_read and diagnostics_before is not None:
                require_diagnostics_healthy(diagnostics_before, diagnostics_after)
            if final_link_read and link_before is not None:
                require_link_healthy(link_before, link_after)
        except Exception as cleanup_error:
            report['passed'] = False
            report['cleanup_error'] = '{}: {}'.format(type(cleanup_error).__name__, cleanup_error)
        span = last_rx - first_rx if first_rx is not None and last_rx is not None else 0
        report['receive'] = {'bytes': received_bytes, 'notifications': notifications,
                             'receive_span_s': span, 'bytes_per_second': received_bytes / span if span else None,
                             'notification_bytes': describe(notification_sizes),
                             'notification_interval_ms': describe(notification_intervals),
                             'queue_highwater': segment_highwater,
                             'connection_queue_highwater': state['highwater']}
        report['bridge_before'] = dict(zip(STATUS_NAMES, before)) if before is not None else None
        last_observed = after if after is not None else before
        report['bridge_final_status_read'] = final_status_read
        report['bridge_last_observed'] = dict(zip(STATUS_NAMES, last_observed)) if last_observed is not None else None
        report['bridge_after'] = dict(zip(STATUS_NAMES, after)) if final_status_read else None
        report['bridge_delta'] = status_delta(before, after) if before is not None and final_status_read else None
        report['diagnostics_supported'] = diagnostics_supported
        report['diagnostics_before'] = diagnostics_before
        report['diagnostics_final_read'] = final_diagnostics_read
        report['diagnostics_last_observed'] = diagnostics_last_observed
        report['diagnostics_after'] = diagnostics_after if final_diagnostics_read else None
        report['diagnostics_delta'] = (diagnostics_delta(diagnostics_before, diagnostics_after)
                                       if diagnostics_before is not None and final_diagnostics_read else None)
        report['link_supported'] = link_supported
        report['link_before'] = link_before
        report['link_final_read'] = final_link_read
        report['link_last_observed'] = link_last_observed
        report['link_last_observed_metrics'] = link_connection_metrics(link_last_observed)
        report['link_after'] = link_after if final_link_read else None
        report['link_delta'] = (link_diagnostics_delta(link_before, link_after)
                                if link_before is not None and final_link_read else None)
        with (directory / (stem + '.json')).open('x', encoding='utf-8') as file:
            json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
        if isinstance(error, asyncio.CancelledError):
            raise error
    return report


async def run(args):
    from bleak import BleakClient, BleakScanner

    directory = args.output_root / datetime.now().strftime('%Y%m%d_%H%M%S_%f_ble_benchmark')
    directory.mkdir(parents=True, exist_ok=False)
    summary = {'schema': 1, 'expected_mouse': args.mouse, 'periods_ms': list(args.periods),
               'duration_per_segment_s': args.duration, 'segments': [], 'passed': False,
               'link_supported': None, 'link_at_connection': None, 'link_last_observed': None}
    summary.update(link_connection_metrics(None))
    incoming = asyncio.Queue(maxsize=512)
    state = {'failure': None, 'highwater': 0}

    def received(characteristic, data):
        try:
            incoming.put_nowait((time.monotonic(), bytes(data)))
            state['highwater'] = max(state['highwater'], incoming.qsize())
        except asyncio.QueueFull:
            state['failure'] = 'PC receive queue overflow'

    try:
        if args.address:
            device = await BleakScanner.find_device_by_address(args.address, timeout=15)
        else:
            device = await BleakScanner.find_device_by_name(args.name, timeout=15)
        if device is None:
            raise BenchmarkError('Mouse bridge not found')
        summary['address'] = device.address
        options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
        async with BleakClient(device, disconnected_callback=lambda _: state.update(failure='BLE disconnected'),
                               **options) as client:
            lock = asyncio.Lock()
            try:
                summary['bridge_at_connection'] = dict(zip(STATUS_NAMES, await read_status(client, lock)))
                try:
                    summary['diagnostics_at_connection'] = await read_diagnostics(client, lock)
                    summary['diagnostics_supported'] = summary['diagnostics_at_connection'] is not None
                    state['diagnostics_supported'] = summary['diagnostics_supported']
                except Exception as diagnostic_error:
                    summary['diagnostics_read_error'] = '{}: {}'.format(type(diagnostic_error).__name__, diagnostic_error)
                    raise
                try:
                    summary['link_at_connection'] = await read_link_diagnostics(client, lock)
                    summary['link_supported'] = summary['link_at_connection'] is not None
                    state['link_supported'] = summary['link_supported']
                except Exception as link_error:
                    summary['link_read_error'] = '{}: {}'.format(type(link_error).__name__, link_error)
                    raise
                summary['link_last_observed'] = summary['link_at_connection']
                summary.update(link_connection_metrics(summary['link_last_observed']))
                await client.start_notify(transport.NUS_TX, received)
                await stream_control(client, lock, stop=True)
                await drain(incoming)
                summary['idle_response'] = await query_idle(client, lock, incoming, args.mouse, args.enter_shell)
                await drain(incoming, args.settle_seconds)
                try:
                    summary['mtu_size'] = client.mtu_size
                except (AttributeError, NotImplementedError):
                    summary['mtu_size'] = None
                summary['mtu_note'] = ('Backend-reported ATT MTU; Windows/WinRT does not establish effective '
                                       'notification payload. Actual connection interval, PHY and data length '
                                       'come from optional HY0020 link diagnostics; unavailable values are null.')
                print('Connected: MTU={} bridge_status={}'.format(
                    summary['mtu_size'], summary['bridge_at_connection']), flush=True)
                for index, period in enumerate(args.periods, 1):
                    print('Testing {} ms for {} s -> {}'.format(period, args.duration, directory), flush=True)
                    report = await capture_segment(client, lock, incoming, state, directory,
                                                   index, period, args.duration)
                    summary['segments'].append(report)
                    summary['link_supported'] = state.get('link_supported')
                    if report.get('link_last_observed') is not None:
                        summary['link_last_observed'] = report['link_last_observed']
                        summary['link_observation_segment'] = index
                        summary.update(link_connection_metrics(summary['link_last_observed']))
                    sample = report.get('sample', {})
                    print('{} ms: {} rows={} lost={} crc={} effective_hz={} mean_dt_ms={}'.format(
                        period, 'PASS' if report['passed'] else 'FAIL',
                        sample.get('rows', 0), sample.get('lost_frames', 0), sample.get('crc_errors', 0),
                        sample.get('effective_hz'), sample.get('interval_ms', {}).get('mean')), flush=True)
                    if not report['passed']:
                        break
                summary['passed'] = len(summary['segments']) == len(args.periods) and all(
                    item['passed'] for item in summary['segments'])
            finally:
                if client.is_connected:
                    try:
                        await asyncio.wait_for(stream_control(client, lock, stop=True), 2)
                        await drain(incoming)
                    except Exception as error:
                        summary['stop_error'] = str(error)
                        summary['passed'] = False
    except (Exception, asyncio.CancelledError) as error:
        summary['error'] = '{}: {}'.format(type(error).__name__, error)
        if isinstance(error, asyncio.CancelledError):
            raise
    finally:
        with (directory / 'summary.json').open('x', encoding='utf-8') as file:
            json.dump(summary, file, ensure_ascii=False, indent=2, allow_nan=False)
        print('Saved benchmark: {}'.format(directory), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE')
    parser.add_argument('--address')
    parser.add_argument('--mouse', choices=('A', 'B'), default='A')
    parser.add_argument('--enter-shell', action='store_true', help='アイドルのモード選択からシェルへ移行する')
    parser.add_argument('--periods', type=parse_periods, default=(20, 10, 5, 4, 3), help='ms、カンマ区切り（既定20,10,5,4,3）')
    parser.add_argument('--duration', type=float, default=10, help='1段あたりの秒数、0より大きく60以下（既定10）')
    parser.add_argument('--settle-seconds', type=float, default=5, help='アイドル確認後のBLE安定待ち、0..15秒（既定5）')
    parser.add_argument('--output-root', type=Path, default=Path(__file__).resolve().parents[1] / 'logs')
    args = parser.parse_args()
    if not 0 < args.duration <= 60 or not 0 <= args.settle_seconds <= 15:
        parser.error('duration must be 0 < seconds <= 60; settle-seconds must be 0..15')
    try:
        summary = asyncio.run(run(args))
    except KeyboardInterrupt:
        print('Stopped; stream stop requested and available results saved.', file=sys.stderr)
        return 130
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
