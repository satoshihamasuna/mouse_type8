"""Read FT232H UART with WinUSB; no COM driver required."""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / '.tools' / 'python'))

import libusb_package
import pyftdi.serialext


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    if libusb_package.get_libusb1_backend() is None:
        raise RuntimeError('libusb backend is not available')
    print('FT232H UART RX: D1, 115200 8N1 (WinUSB)', flush=True)
    with pyftdi.serialext.serial_for_url(
        'ftdi://ftdi:232h/1', baudrate=115200, bytesize=8,
        parity='N', stopbits=1, timeout=0.5,
        rtscts=False, xonxoff=False, dsrdtr=False
    ) as port:
        deadline = time.monotonic() + args.seconds
        received = 0
        while time.monotonic() < deadline:
            data = port.read(256)
            if data:
                received += len(data)
                print(data.decode('ascii', errors='replace'), end='', flush=True)
        print('\nReceived {} bytes'.format(received), flush=True)


if __name__ == '__main__':
    main()
