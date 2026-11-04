"""Continuously capture all mouse_type8 log fields over BLE into a CSV file."""
import argparse
import asyncio
import binascii
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import struct
import sys

import mouse_ble_gateway as transport


MIN_PERIOD_MS = 3

LOG_LABELS = (
    'ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo',
    'ideal.length', 'ego.length', 'ideal.radian', 'ego.radian',
    'V_r', 'V_l', 'sp_feedback', 'sp_feedforward', 'om_feedback', 'om_feedforward',
    'sen_l.avg_distance', 'sen_r.avg_distance', 'sen_l.distance', 'sen_r.distance',
    'ego.x_point', 'ideal.x_point', 'ego.turn_x', 'ideal.turn_x', 'ego.turn_y', 'ideal.turn_y',
    'ego.turn_slip_theta', 'ideal.turn_slip_theta', 'ego.horizon_accel.get',
    'ideal.horizon_accel.get', 'ego.horizon_velo.get', 'ideal.horizon_velo.get',
    'Encoder_GetProperty_Right().sp_pulse', 'Encoder_GetProperty_Left().sp_pulse',
    'Battery', 'ideal.accel', 'ego.accel', 'speed_ctrl.get_I_peration',
    'omega_ctrl.get_I_peration', 'control_ir', 'control_ir_dot', 'sen_l.error',
    'sen_r.error', 'sen_fl.distance', 'sen_fr.distance', 'speed_ctrl.get_D_peration',
    'omega_ctrl.get_D_peration', 'speed_ctrl.Kp', 'speed_ctrl.Ki', 'speed_ctrl.Kd',
    'omega_ctrl.Kp', 'omega_ctrl.Ki', 'omega_ctrl.Kd',
    'irq.total_us', 'irq.logging_us', 'irq.max_us', 'irq.period_us',
)
MAGIC = b'L8S2'
FRAME = struct.Struct('<4sIIH55eH')
FRAMES = {b'L8S1': struct.Struct('<4sIIH51eH'), MAGIC: FRAME}
CSV_HEADER = ('cnt', *LOG_LABELS, 'sequence', 'time_ms', 'period_ms', 'lost_frames')


def control_packet(period_ms, *, stop=False, action=None):
    if not MIN_PERIOD_MS <= period_ms <= 1000:
        raise ValueError('period must be between {} and 1000 ms'.format(MIN_PERIOD_MS))
    action = (0 if stop else 1) if action is None else action
    if action not in (0, 1, 2, 3):
        raise ValueError('unknown stream/exploration action')
    packet = struct.pack('<3sBH', b'\x00L8', action, period_ms)
    return packet + struct.pack('<H', binascii.crc_hqx(packet, 0xffff))


@dataclass
class Sample:
    sequence: int
    time_ms: int
    period_ms: int
    values: tuple


class StreamDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data, text_sink=None):
        """Resynchronize across notification splits, text, and damaged frames."""
        self.buffer.extend(data)
        samples = []
        while True:
            positions = [self.buffer.find(magic) for magic in FRAMES]
            start = min((position for position in positions if position >= 0), default=-1)
            if start < 0:
                keep = next((n for n in (3, 2, 1) if self.buffer.endswith(MAGIC[:n])), 0)
                length = len(self.buffer) - keep
                if text_sink and length:
                    text_sink(bytes(self.buffer[:length]))
                del self.buffer[:length]
                break
            if text_sink and start:
                text_sink(bytes(self.buffer[:start]))
            del self.buffer[:start]
            frame = FRAMES[bytes(self.buffer[:4])]
            if len(self.buffer) < frame.size:
                break
            checksum = struct.unpack_from('<H', self.buffer, frame.size - 2)[0]
            if binascii.crc_hqx(self.buffer[:frame.size - 2], 0xffff) != checksum:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            _, sequence, time_ms, period_ms, *values = frame.unpack_from(self.buffer)
            if not MIN_PERIOD_MS <= period_ms <= 1000:
                del self.buffer[0]
                continue
            samples.append(Sample(sequence, time_ms, period_ms, tuple(values[:-1])))
            del self.buffer[:frame.size]
        return samples


class CsvCapture:
    def __init__(self, file):
        self.file = file
        self.writer = csv.writer(file)
        self.field_count = None
        self.decoder = StreamDecoder()
        self.rows = 0
        self.lost = 0
        self.previous_sequence = None
        self.previous_time = None
        self.elapsed_ms = 0
        self.latest = None

    def feed(self, data, text_sink=None):
        samples = self.decoder.feed(data, text_sink)
        for sample in samples:
            if self.field_count is None:
                self.field_count = len(sample.values)
                self.writer.writerow(('cnt', *LOG_LABELS[:self.field_count],
                                      'sequence', 'time_ms', 'period_ms', 'lost_frames'))
            elif len(sample.values) != self.field_count:
                raise RuntimeError('Log field count changed; reconnect to start a new CSV')
            if self.previous_sequence is None:
                lost = 0  # Earlier attempts may belong to a previous BLE session.
            else:
                lost = (sample.sequence - self.previous_sequence - 1) & 0xffffffff
                if lost >= 0x80000000:
                    raise RuntimeError('Log stream restarted or sequence went backwards')
                self.elapsed_ms += (sample.time_ms - self.previous_time) & 0xffffffff
            self.lost += lost
            self.writer.writerow((self.elapsed_ms, *sample.values, sample.sequence,
                                  sample.time_ms, sample.period_ms, lost))
            self.previous_sequence = sample.sequence
            self.previous_time = sample.time_ms
            self.rows += 1
            self.latest = sample
        if samples:
            self.file.flush()
        return len(samples)


async def run(args):
    from bleak import BleakClient, BleakScanner

    if args.address:
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
    else:
        device = await BleakScanner.find_device_by_name(args.name, timeout=15)
    if device is None:
        raise RuntimeError('Mouse bridge not found')
    disconnected = asyncio.Event()
    incoming = asyncio.Queue(maxsize=512)
    failure = None

    def received(characteristic, data):
        nonlocal failure
        try:
            incoming.put_nowait(bytes(data))
        except asyncio.QueueFull:
            failure = RuntimeError('PC receive queue overflow')
            disconnected.set()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', newline='', encoding='utf-8') as file:
        capture = CsvCapture(file)
        options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
        async with BleakClient(device, disconnected_callback=lambda _: disconnected.set(), **options) as client:
            baseline = transport.decode_status(await client.read_gatt_char(transport.STATUS))
            await client.start_notify(transport.NUS_TX, received)
            lock = asyncio.Lock()
            loop = asyncio.get_running_loop()
            started = last_sample = loop.time()
            next_request = next_status = started
            print('Connected: {} ms / {} Hz -> {}'.format(
                args.period_ms, 1000 / args.period_ms, args.output), flush=True)
            try:
                while args.duration == 0 or loop.time() - started < args.duration:
                    if disconnected.is_set():
                        raise failure or ConnectionError('BLE disconnected')
                    now = loop.time()
                    if now >= next_request:
                        await transport.write_bytes(client, control_packet(args.period_ms), lock)
                        next_request = loop.time() + 0.5
                    if now >= next_status:
                        current = transport.decode_status(await client.read_gatt_char(transport.STATUS))
                        transport.check_status(baseline, current)
                        print('rows={} lost={} crc_errors={}'.format(
                            capture.rows, capture.lost, capture.decoder.crc_errors), flush=True)
                        next_status = loop.time() + 1
                    try:
                        data = await asyncio.wait_for(incoming.get(), timeout=0.1)
                    except asyncio.TimeoutError:
                        data = b''
                    if capture.feed(data):
                        last_sample = loop.time()
                    if loop.time() - last_sample >= 5:
                        raise TimeoutError('No valid log frames for 5 s; flash STM32 firmware with log streaming')
                if capture.rows == 0:
                    raise TimeoutError('No log samples received')
                current = transport.decode_status(await client.read_gatt_char(transport.STATUS))
                transport.check_status(baseline, current)
            finally:
                if not disconnected.is_set():
                    try:
                        await asyncio.wait_for(transport.write_bytes(
                            client, control_packet(args.period_ms, stop=True), lock), timeout=1)
                    except Exception:
                        pass  # The STM32 lease also stops the stream after 3 seconds.
                while not incoming.empty():
                    capture.feed(incoming.get_nowait())
                print('Saved {} rows, lost={}, crc_errors={}: {}'.format(
                    capture.rows, capture.lost, capture.decoder.crc_errors, args.output), flush=True)
    return capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE')
    parser.add_argument('--address')
    parser.add_argument('--period-ms', type=int, default=20, help='{}..1000 ms (default: 20 / 50 Hz)'.format(MIN_PERIOD_MS))
    parser.add_argument('--duration', type=float, default=0, help='seconds; 0 means until Ctrl+C')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] /
                        'logs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_ble_live.csv'))
    args = parser.parse_args()
    if sys.version_info < (3, 10):
        parser.error('Python 3.10+ is required')
    if not MIN_PERIOD_MS <= args.period_ms <= 1000 or not 0 <= args.duration < float('inf'):
        parser.error('period must be {}..1000 ms and duration must be finite and nonnegative'.format(MIN_PERIOD_MS))
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('Stopped. Received rows have been saved.')
    except Exception as error:
        print('{}: {}'.format(type(error).__name__, error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
