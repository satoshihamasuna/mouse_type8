"""Expose the HY0020 mouse UART bridge as a local, binary-safe TCP stream."""
import argparse
import asyncio
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / '.tools' / 'ble_python'))

NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
STATUS = '6e400004-b5a3-f393-e0a9-e50e24dcca9e'
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


async def run(args):
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

    def received(characteristic, data):
        nonlocal failure
        if active_writer is None or active_writer.is_closing():
            return  # Do not replay bytes generated before a GUI session.
        if active_writer.transport.get_write_buffer_size() + len(data) > 4 * 1024 * 1024:
            failure = RuntimeError('Local client is not draining its receive buffer')
            active_writer.close()
            client_done.set()
            return
        active_writer.write(data)  # No text decoding or newline conversion.

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
                while data := await reader.read(1024):
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
    print('GUI session ended; BLE disconnected', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE')
    parser.add_argument('--address', help='optional Bluetooth address')
    parser.add_argument('--port', type=int, default=8765, help='localhost TCP port')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
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
