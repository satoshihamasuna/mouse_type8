"""Send mouse_type8 shell commands directly over HY0020's BLE UART bridge."""
import argparse
import asyncio
import codecs
from pathlib import Path
import sys
import threading

import mouse_ble_gateway as transport


async def send_command(client, command, lock, char_delay):
    # Match the upstream GUI: ASCII command, one CR, paced character writes.
    payload = command.encode('ascii') + b'\r'
    for byte in payload:
        await transport.write_bytes(client, bytes([byte]), lock)
        if char_delay:
            await asyncio.sleep(char_delay)


async def wait_response(last_rx, start, timeout, idle, disconnected):
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        if disconnected.is_set():
            raise ConnectionError('BLE disconnected')
        if last_rx[0] >= start and loop.time() - last_rx[0] >= idle:
            return
        await asyncio.sleep(0.05)
    raise TimeoutError('No complete response within --response-timeout')


async def run(args):
    from bleak import BleakClient, BleakScanner

    print('Searching for {}...'.format(args.name), file=sys.stderr, flush=True)
    if args.address:
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
    else:
        device = await BleakScanner.find_device_by_name(args.name, timeout=15)
    if device is None:
        raise RuntimeError('Mouse bridge not found. Check firmware, power, and other BLE clients.')
    disconnected = asyncio.Event()
    last_rx = [float('-inf')]
    decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
    request_lock = asyncio.Lock()
    failure = None
    raw_file = args.raw_output.open('xb') if args.raw_output else None

    def received(characteristic, data):
        last_rx[0] = asyncio.get_running_loop().time()
        if raw_file:
            raw_file.write(data)
            raw_file.flush()
        print(decoder.decode(data), end='', flush=True)

    options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
    try:
        async with BleakClient(device, disconnected_callback=lambda _: disconnected.set(), **options) as client:
            baseline = transport.decode_status(await client.read_gatt_char(transport.STATUS))
            await client.start_notify(transport.NUS_TX, received)
            print('Connected. Mouse must be in shell mode. /quit to disconnect.', file=sys.stderr, flush=True)

            async def health_check():
                nonlocal failure
                try:
                    while True:
                        await asyncio.sleep(1)
                        async with request_lock:
                            current = transport.decode_status(await client.read_gatt_char(transport.STATUS))
                        transport.check_status(baseline, current)
                except asyncio.CancelledError:
                    raise
                except Exception as error:
                    failure = error
                    disconnected.set()

            health_task = asyncio.create_task(health_check())
            try:
                if args.enter_shell:
                    # Mode::Select_Mode accepts shell + CR without an acknowledgement.
                    await send_command(client, 'shell', request_lock, args.char_delay)
                    await asyncio.sleep(0.5)
                if args.command:
                    for command in args.command:
                        start = asyncio.get_running_loop().time()
                        print('\n> {}'.format(command), file=sys.stderr, flush=True)
                        await send_command(client, command, request_lock, args.char_delay)
                        try:
                            await wait_response(last_rx, start, args.response_timeout,
                                                args.idle_timeout, disconnected)
                        except ConnectionError:
                            if failure:
                                raise failure
                            raise
                else:
                    commands = asyncio.Queue()
                    loop = asyncio.get_running_loop()

                    def read_stdin():
                        try:
                            for line in sys.stdin:
                                if loop.is_closed():
                                    return
                                loop.call_soon_threadsafe(commands.put_nowait, line.rstrip('\r\n'))
                            if not loop.is_closed():
                                loop.call_soon_threadsafe(commands.put_nowait, None)
                        except (OSError, RuntimeError):
                            return

                    threading.Thread(target=read_stdin, daemon=True).start()
                    while not disconnected.is_set():
                        try:
                            command = await asyncio.wait_for(commands.get(), timeout=0.2)
                        except asyncio.TimeoutError:
                            continue
                        if command is None or command == '/quit':
                            break
                        try:
                            command.encode('ascii')
                        except UnicodeEncodeError:
                            print('Commands must be ASCII; nothing sent.', file=sys.stderr)
                            continue
                        await send_command(client, command, request_lock, args.char_delay)
                if failure:
                    raise failure
                if disconnected.is_set():
                    raise ConnectionError('BLE disconnected; commands were not replayed')
            finally:
                health_task.cancel()
                await asyncio.gather(health_task, return_exceptions=True)
    finally:
        if raw_file:
            raw_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE')
    parser.add_argument('--address')
    parser.add_argument('--command', action='append', help='one ASCII shell command; repeat for a batch')
    parser.add_argument('--enter-shell', action='store_true', help='send shell + CR to enter shell from idle mode selection')
    parser.add_argument('--char-delay', type=float, default=0.08, help='seconds between bytes (default: 0.08)')
    parser.add_argument('--response-timeout', type=float, default=60)
    parser.add_argument('--idle-timeout', type=float, default=1)
    parser.add_argument('--raw-output', type=Path, help='save exact received bytes to a new file')
    args = parser.parse_args()
    if sys.version_info < (3, 10):
        parser.error('Python 3.10+ is required')
    if not 0 <= args.char_delay <= 10 or not 0 < args.idle_timeout <= args.response_timeout < float('inf'):
        parser.error('Invalid delay/timeout values')
    for command in args.command or []:
        try:
            command.encode('ascii')
        except UnicodeEncodeError:
            parser.error('--command must be ASCII')
        if '\r' in command or '\n' in command:
            parser.error('Use repeated --command options instead of embedded newlines')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
    except Exception as error:
        print('{}: {}'.format(type(error).__name__, error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
