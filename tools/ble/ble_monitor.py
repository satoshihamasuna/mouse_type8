"""Receive HY0020 NUS notifications using the PC's Bluetooth adapter."""
import argparse
import asyncio
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / '.tools' / 'ble_python'))

NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'


def positive_seconds(value):
    number = float(value)
    if not 0 < number < float('inf'):
        raise argparse.ArgumentTypeError('must be a positive, finite number')
    return number


async def run(args):
    from bleak import BleakClient, BleakScanner

    if args.scan:
        print('Scanning for Nordic UART Service devices...', flush=True)
        devices = await BleakScanner.discover(
            timeout=args.timeout, return_adv=True, service_uuids=[NUS_SERVICE]
        )
        for device, advertisement in devices.values():
            print('{}  {}'.format(device.address, advertisement.local_name or device.name or '(unnamed)'))
        print('Found {} NUS device(s)'.format(len(devices)), flush=True)
        return 0

    identifier = args.address or args.name
    print('Searching for {}...'.format(identifier), flush=True)
    if args.address:
        device = await BleakScanner.find_device_by_address(args.address, timeout=args.timeout)
    else:
        device = await BleakScanner.find_device_by_name(args.name, timeout=args.timeout)
    if device is None:
        print('Device not found. Check BLE firmware, power, and phone disconnection.', file=sys.stderr)
        return 1

    disconnected = asyncio.Event()
    received = 0

    def on_notification(characteristic, data):
        nonlocal received
        received += len(data)
        print(data.decode('utf-8', errors='replace').replace('\r\n', '\n'), end='', flush=True)

    options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
    async with BleakClient(
        device, disconnected_callback=lambda client: disconnected.set(), **options
    ) as client:
        await client.start_notify(NUS_TX, on_notification)
        print('Connected to {}; receiving NUS text'.format(device.address), flush=True)
        if args.send is not None:
            await client.write_gatt_char(NUS_RX, args.send.encode('utf-8'), response=True)
            print('Sent: {}'.format(args.send), flush=True)
        try:
            await asyncio.wait_for(disconnected.wait(), timeout=args.seconds)
        except asyncio.TimeoutError:
            print('\nReceived {} bytes'.format(received), flush=True)
            return 0
        print('\nDisconnected before the receive interval ended.', file=sys.stderr)
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='HY0020-MOUSE', help='advertised name (default: HY0020-MOUSE)')
    parser.add_argument('--address', help='connect by Bluetooth address instead of name')
    parser.add_argument('--scan', action='store_true', help='list nearby NUS devices without connecting')
    parser.add_argument('--timeout', type=positive_seconds, default=15, help='scan timeout in seconds')
    parser.add_argument('--seconds', type=positive_seconds, default=30, help='receive interval in seconds')
    parser.add_argument('--send', help='also send one UTF-8 string to HY0020 (1-20 bytes)')
    args = parser.parse_args()
    if args.scan and args.send is not None:
        parser.error('--send cannot be combined with --scan')
    if args.send is not None and not 1 <= len(args.send.encode('utf-8')) <= 20:
        parser.error('--send must be 1-20 bytes in UTF-8')
    if sys.version_info < (3, 10):
        parser.error('Python 3.10+ is required; use the SDK Python shown in the README')
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return 0
    except ImportError as error:
        print('Missing BLE dependency: {}. Install tools/ble/requirements-ble.txt.'.format(error), file=sys.stderr)
        return 1
    except Exception as error:
        if (getattr(error, 'winerror', 0) or 0) & 0xFFFFFFFF == 0x80070422:
            print('Bluetooth service is disabled. Enable Bluetooth Support Service '
                  '(bthserv), then turn Bluetooth on in Windows Settings.', file=sys.stderr)
            return 1
        print('BLE error: {}: {}'.format(type(error).__name__, error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
