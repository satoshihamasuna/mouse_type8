"""Expose the HY0020 mouse UART bridge as a local, binary-safe TCP stream."""
import argparse
import asyncio
import contextlib
import json
from pathlib import Path
import struct
import sys

dependency_root = Path(__file__).resolve().parents[2] / '.tools'
for directory in ('ble_python', 'ble_python{}{}'.format(*sys.version_info[:2])):
    sys.path.insert(0, str(dependency_root / directory))

NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
STATUS = '6e400004-b5a3-f393-e0a9-e50e24dcca9e'
DIAGNOSTICS = '6e400005-b5a3-f393-e0a9-e50e24dcca9e'
DIAGNOSTICS_FIELDS = (
    'version', 'last_stop_reason', 'stop_reason_or', 'overrun_count', 'parity_count',
    'framing_count', 'break_count', 'collision_count', 'noise_count', 'other_reason_count',
    'rx_disabled_count', 'rx_restart_attempts', 'rx_restart_failures',
    'rx_buffer_alloc_failures', 'rx_buffer_rsp_failures', 'rx_buf_request_events',
    'rx_buffer_size', 'rx_buffer_count', 'rx_timeout_us', 'rx_mode_flags',
)
DIAGNOSTICS_COUNTER_FIELDS = (
    'overrun_count', 'parity_count', 'framing_count', 'break_count', 'collision_count',
    'noise_count', 'other_reason_count', 'rx_disabled_count', 'rx_restart_attempts',
    'rx_restart_failures', 'rx_buffer_alloc_failures', 'rx_buffer_rsp_failures',
    'rx_buf_request_events',
)
LINK_DIAGNOSTICS = '6e400006-b5a3-f393-e0a9-e50e24dcca9e'
LINK_DIAGNOSTICS_FIELDS = (
    'version', 'mtu', 'interval_us', 'latency', 'supervision_timeout_ms',
    'tx_phy', 'rx_phy', 'tx_max_len', 'tx_max_time_us', 'rx_max_len', 'rx_max_time_us',
    'ring_bytes', 'ring_highwater', 'notify_calls', 'notify_bytes', 'notify_failures',
    'notify_call_max_ms', 'notify_retry_events', 'batch_flush_ms', 'sdc_tx_packet_count',
    'acl_tx_count', 'acl_tx_size', 'ring_capacity', 'flags',
)
LINK_DIAGNOSTICS_COUNTER_FIELDS = (
    'notify_calls', 'notify_bytes', 'notify_failures', 'notify_retry_events',
)
HOST = '127.0.0.1'


async def write_bytes(client, data, request_lock, *, retry_timeout=10.0):
    """Retry only explicit ATT resource rejection, never ambiguous timeouts."""
    from bleak.exc import BleakGATTProtocolError

    for offset in range(0, len(data), 20):
        chunk = data[offset:offset + 20]
        deadline = asyncio.get_running_loop().time() + retry_timeout
        while True:
            try:
                async with request_lock:
                    await client.write_gatt_char(NUS_RX, chunk, response=True)
                break
            except BleakGATTProtocolError as error:
                if int(error.code) != 0x11 or asyncio.get_running_loop().time() >= deadline:
                    raise
                await asyncio.sleep(0.02)


def decode_status(payload):
    if len(payload) != 16:
        raise ValueError('Unexpected bridge status length; install mouse_uart_ble firmware')
    return struct.unpack('<IIII', payload)


def check_status(initial, current):
    # Busy writes were explicitly rejected and can be retried; other errors lose data.
    if any(current[index] != initial[index] for index in (0, 1, 3)):
        raise RuntimeError('UART/BLE data loss detected: dropped={}, UART errors={}, BLE errors={}'.format(
            current[0], current[1], current[3]))


def decode_diagnostics(payload):
    """Decode optional version-one diagnostics without changing status semantics."""
    if len(payload) != 80:
        raise ValueError('Unexpected bridge diagnostics length; expected 80 bytes')
    values = struct.unpack('<20I', payload)
    if values[0] != 1:
        raise ValueError('Unsupported bridge diagnostics version: {}'.format(values[0]))
    return dict(zip(DIAGNOSTICS_FIELDS, values))


async def read_diagnostics(client, request_lock):
    """Return None only when the peer explicitly lacks the optional characteristic."""
    from bleak.exc import BleakCharacteristicNotFoundError, BleakGATTProtocolError

    try:
        async with request_lock:
            payload = await client.read_gatt_char(DIAGNOSTICS)
    except BleakCharacteristicNotFoundError:
        return None
    except BleakGATTProtocolError as error:
        if int(error.code) == 0x0A:  # ATT Attribute Not Found.
            return None
        raise
    return decode_diagnostics(payload)


def decode_link_diagnostics(payload):
    """Decode negotiated-link and relay metrics; zero negotiated values are unknown."""
    if len(payload) != 96:
        raise ValueError('Unexpected bridge link diagnostics length; expected 96 bytes')
    values = struct.unpack('<24I', payload)
    if values[0] != 1:
        raise ValueError('Unsupported bridge link diagnostics version: {}'.format(values[0]))
    return dict(zip(LINK_DIAGNOSTICS_FIELDS, values))


async def read_link_diagnostics(client, request_lock):
    """Return None only for explicit absence of the optional link characteristic."""
    from bleak.exc import BleakCharacteristicNotFoundError, BleakGATTProtocolError

    try:
        async with request_lock:
            payload = await client.read_gatt_char(LINK_DIAGNOSTICS)
    except BleakCharacteristicNotFoundError:
        return None
    except BleakGATTProtocolError as error:
        if int(error.code) == 0x0A:
            return None
        raise
    return decode_link_diagnostics(payload)


class GuiCapture:
    """Record live frames while forwarding non-stream UART bytes unchanged."""
    def __init__(self, output, period_ms):
        from mouse_ble_log import CsvCapture, control_packet
        control_packet(period_ms)
        self.output, self.period_ms = Path(output), period_ms
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.files = contextlib.ExitStack()
        try:
            csv_file = self.files.enter_context(self.output.open('x', newline='', encoding='utf-8'))
            self.raw = self.files.enter_context(self.output.with_suffix('.raw.bin').open('xb'))
            self.shell = self.files.enter_context(self.output.with_suffix('.shell.bin').open('xb'))
            self.capture = CsvCapture(csv_file)
        except BaseException:
            self.files.close()
            raise

    def feed(self, data, forward):
        self.raw.write(data)
        def text(data):
            self.shell.write(data)
            forward(data)
        self.capture.feed(data, text)

    def report(self):
        latest = self.capture.latest
        status = {'period_ms': self.period_ms, 'rows': self.capture.rows,
                  'lost': self.capture.lost, 'crc_errors': self.capture.decoder.crc_errors,
                  'latest_sequence': latest.sequence if latest else None,
                  'latest_time_ms': latest.time_ms if latest else None}
        temporary = self.output.with_suffix('.status.tmp')
        temporary.write_text(json.dumps(status, indent=2), encoding='utf-8')
        temporary.replace(self.output.with_suffix('.status.json'))
        self.raw.flush()
        self.shell.flush()
        return status

    def close(self):
        try:
            self.report()
        finally:
            self.files.close()


async def run(args):
    output = getattr(args, 'capture_output', None)
    recorder = GuiCapture(output, getattr(args, 'capture_period_ms', 5)) if output else None
    try:
        return await run_link(args, recorder)
    finally:
        if recorder:
            recorder.close()


async def run_link(args, recorder=None):
    from bleak import BleakClient, BleakScanner

    print('Searching for {}...'.format(args.name), flush=True)
    if args.address:
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
    else:
        device = await BleakScanner.find_device_by_name(args.name, timeout=15)
    if device is None:
        raise RuntimeError('Mouse bridge not found. Check firmware, 3.3 V power, and other BLE clients.')

    disconnected = asyncio.Event()
    client_done = asyncio.Event()
    active_writer = None
    session_task = None
    failure = None
    request_lock = asyncio.Lock()

    def forward(data):
        nonlocal failure
        if active_writer is None or active_writer.is_closing():
            return  # Do not replay bytes generated before a GUI session.
        if active_writer.transport.get_write_buffer_size() + len(data) > 4 * 1024 * 1024:
            failure = RuntimeError('Local client is not draining its receive buffer')
            active_writer.close()
            client_done.set()
            return
        active_writer.write(data)  # No text decoding or newline conversion.

    def received(characteristic, data):
        nonlocal failure
        try:
            if recorder:
                recorder.feed(data, forward)
            else:
                forward(data)
        except Exception as error:
            failure = error
            client_done.set()

    options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
    async with BleakClient(device, disconnected_callback=lambda _: disconnected.set(), **options) as client:
        initial = decode_status(await client.read_gatt_char(STATUS))
        await client.start_notify(NUS_TX, received)

        async def serve_gui(reader, writer):
            nonlocal active_writer, session_task, failure
            if active_writer is not None:
                writer.close()
                await writer.wait_closed()
                return
            active_writer = writer
            session_task = asyncio.current_task()
            print('GUI connected; forwarding raw UART bytes', flush=True)
            try:
                while True:
                    try:
                        data = await reader.read(1024)
                    except ConnectionResetError:
                        break  # Windows GUI close can reset its localhost socket.
                    if not data:
                        break
                    await write_bytes(client, data, request_lock)
            except Exception as error:
                failure = error
            finally:
                writer.close()
                try:
                    await writer.wait_closed()
                except (ConnectionError, OSError):
                    pass
                client_done.set()

        async def watch_health():
            while True:
                if recorder:
                    from mouse_ble_log import control_packet
                    await write_bytes(client, control_packet(recorder.period_ms), request_lock)
                    recorder.report()
                await asyncio.sleep(1)
                async with request_lock:
                    current = decode_status(await client.read_gatt_char(STATUS))
                check_status(initial, current)

        server = await asyncio.start_server(serve_gui, HOST, args.port)
        health_task = asyncio.create_task(watch_health())
        disconnected_task = asyncio.create_task(disconnected.wait())
        done_task = asyncio.create_task(client_done.wait())
        try:
            print('Ready: socket://{}:{} (one GUI client; Ctrl+C to stop)'.format(HOST, args.port), flush=True)
            done, _ = await asyncio.wait(
                [health_task, disconnected_task, done_task], return_when=asyncio.FIRST_COMPLETED
            )
            if health_task in done:
                health_task.result()
            if disconnected_task in done:
                raise RuntimeError('BLE disconnected; in-flight data will not be replayed')
            if failure:
                raise failure
        finally:
            server.close()
            await server.wait_closed()
            if session_task is not None and not session_task.done():
                session_task.cancel()
                await asyncio.gather(session_task, return_exceptions=True)
            if active_writer:
                active_writer.close()
                try:
                    await active_writer.wait_closed()
                except (ConnectionError, OSError):
                    pass
            for task in (health_task, disconnected_task, done_task):
                task.cancel()
            await asyncio.gather(health_task, disconnected_task, done_task, return_exceptions=True)
            if recorder and not disconnected.is_set():
                from mouse_ble_log import control_packet
                try:
                    await asyncio.wait_for(write_bytes(client, control_packet(recorder.period_ms, stop=True), request_lock), 1)
                except Exception:
                    pass  # The firmware stream lease also expires after disconnect.
    print('GUI session ended; BLE disconnected', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE')
    parser.add_argument('--address', help='optional Bluetooth address')
    parser.add_argument('--port', type=int, default=8765, help='localhost TCP port')
    parser.add_argument('--capture-output', type=Path, help='record live frames to a new CSV while the GUI remains usable')
    parser.add_argument('--capture-period-ms', type=int, default=5, help='live capture period, 3..1000 ms (default: 5)')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if not 3 <= args.capture_period_ms <= 1000:
        parser.error('--capture-period-ms must be between 3 and 1000')
    if sys.version_info < (3, 10):
        parser.error('Use Python 3.10+ with tools/ble/requirements-ble.txt installed')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
    except Exception as error:
        if (getattr(error, 'winerror', 0) or 0) & 0xFFFFFFFF == 0x80070422:
            print('Bluetooth Support Service (bthserv) is disabled.', file=sys.stderr)
        else:
            print('{}: {}'.format(type(error).__name__, error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
