"""Launch an existing mouse_type8 GUI with a virtual BLE serial port."""
import argparse
from pathlib import Path
import runpy
import sys

GUI_FILES = {
    'main': 'myshell_gui.py',
    'debug': 'myshell_debug.py',
    'turnpattern': 'myshell_turnpattern.py',
    'search': 'myshell_search_gui.py',
}


def install_ble_port(port):
    """Route only the synthetic BLE port; real COM ports keep their behavior."""
    import serial
    from serial.tools import list_ports
    from serial.tools.list_ports_common import ListPortInfo

    original_serial = serial.Serial
    original_ports = list_ports.comports

    def serial_factory(device=None, *args, **kwargs):
        if device is None and 'port' in kwargs:
            device = kwargs.pop('port')
        if str(device).upper() == 'BLE':
            return serial.serial_for_url('socket://127.0.0.1:{}'.format(port), *args, **kwargs)
        return original_serial(device, *args, **kwargs)

    def ports_with_ble(*args, **kwargs):
        virtual = ListPortInfo('BLE')
        virtual.description = 'HY0020 mouse BLE gateway'
        virtual.hwid = 'LOCAL-BLE-NUS'
        return [virtual] + list(original_ports(*args, **kwargs))

    serial.Serial = serial_factory
    list_ports.comports = ports_with_ble


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mouse-repo', type=Path, default=Path(__file__).resolve().parents[2],
                        help='path to mouse_type8 (default: this repository)')
    parser.add_argument('--gui', choices=GUI_FILES, default='main')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    script = args.mouse_repo.resolve() / 'tools' / GUI_FILES[args.gui]
    if not script.is_file():
        parser.error('GUI script not found: {}'.format(script))
    install_ble_port(args.port)
    sys.path.insert(0, str(script.parent))
    sys.argv = [str(script)]
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
