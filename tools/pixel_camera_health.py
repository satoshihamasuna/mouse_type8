"""ADBの端末選択とdumpsys batteryだけで撮影前の残量を読む。アプリ・録画・設定は変更しない。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess


STATUS_NAMES = {1: 'unknown', 2: 'charging', 3: 'discharging', 4: 'not_charging', 5: 'full'}
POWER_FLAGS = (('AC powered', 1, 'ac'), ('USB powered', 2, 'usb'),
               ('Wireless powered', 4, 'wireless'), ('Dock powered', 8, 'dock'))
THRESHOLD_NOTE = 'min_levelは可変の運用上の充電余裕であり、機種の物理的な撮影性能・電池寿命の保証値ではない。'


def parse_adb_devices(output):
    """devices出力の状態も保持し、未認証・offlineを選択済みと扱わない。"""
    devices = []
    for line in output.splitlines():
        line = line.strip()
        if not line or line.startswith('List of devices attached') or line.startswith('*'):
            continue
        parts = line.split()
        if len(parts) < 2:
            raise ValueError('ADB devices出力の形式が不明')
        if parts[0] in {device['serial'] for device in devices}:
            raise ValueError('ADB devices出力にserialの重複')
        devices.append(dict(serial=parts[0], state=parts[1]))
    return devices


def select_device(devices, serial=None):
    if serial is None:
        if len(devices) != 1:
            raise ValueError('ADB_DEVICE_SELECTION: 接続端末を1台にするか--serialを指定してください')
        selected = devices[0]
    else:
        matching = [device for device in devices if device['serial'] == serial]
        if len(matching) != 1:
            raise ValueError('ADB_SERIAL_NOT_FOUND: 指定serialの端末を確認できません')
        selected = matching[0]
    if selected['state'] != 'device':
        raise ValueError('ADB_DEVICE_NOT_READY: ' + selected['state'])
    return selected['serial']


def parse_battery(output):
    """Android BatteryServiceの実値を解釈する。不明・欠損・テスト固定値は拒否する。"""
    if re.search(r'UPDATES\s+STOPPED|mUpdatesStopped\s*[:=]\s*true', output, re.I):
        raise ValueError('BATTERY_UPDATES_STOPPED: テスト用固定値を実測として扱いません')
    values = {}
    for line in output.splitlines():
        match = re.match(r'^\s*([^:]+):\s*(.*?)\s*$', line)
        if match:
            key, value = match.groups()
            key = key.strip()
            if key in values and values[key] != value:
                raise ValueError('BATTERY_DUPLICATE_FIELD: ' + key)
            values[key] = value

    def integer(name):
        value = values.get(name, '')
        if not re.fullmatch(r'[+-]?\d+', value):
            raise ValueError('BATTERY_MISSING_OR_UNKNOWN: ' + name)
        return int(value)

    def boolean(name):
        value = values.get(name, '').lower()
        if value not in ('true', 'false'):
            raise ValueError('BATTERY_MISSING_OR_UNKNOWN: ' + name)
        return value == 'true'

    if not boolean('present'):
        raise ValueError('BATTERY_NOT_PRESENT')
    level, scale, status = integer('level'), integer('scale'), integer('status')
    if scale <= 0 or not 0 <= level <= scale:
        raise ValueError('BATTERY_INVALID_LEVEL_OR_SCALE')
    if status not in STATUS_NAMES or status == 1:
        raise ValueError('BATTERY_STATUS_UNKNOWN')
    voltage, temperature = integer('voltage'), integer('temperature')
    if voltage <= 0:
        raise ValueError('BATTERY_VOLTAGE_UNKNOWN')
    # Android通常出力は電源別boolean。plugged bitmask出力も独立に照合する。
    has_flags = any(name in values for name, _, _ in POWER_FLAGS)
    if has_flags:
        plugged = 0
        for name, bit, _ in POWER_FLAGS:
            if name == 'Dock powered' and name not in values:
                continue
            if boolean(name):
                plugged |= bit
        if 'plugged' in values and integer('plugged') != plugged:
            raise ValueError('BATTERY_POWER_SOURCE_MISMATCH')
    else:
        plugged = integer('plugged')
    if plugged < 0 or plugged & ~15:
        raise ValueError('BATTERY_POWER_SOURCE_UNKNOWN')
    return dict(level=level, scale=scale, level_percent=100.0*level/scale,
                status=status, status_name=STATUS_NAMES[status], plugged=plugged,
                power_sources=[name for _, bit, name in POWER_FLAGS if plugged & bit],
                voltage_mv=voltage, temperature_deci_c=temperature, temperature_c=temperature/10.0,
                present=True)


def _adb_read(command):
    return subprocess.run(command, capture_output=True, text=True, timeout=10,
                          creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)


def inspect_camera_health(adb=None, serial=None, min_level=20):
    """read-onlyなdevices/dumpsys batteryを実行する。不明時はpassed=False。"""
    failures = []
    result = dict(schema_version=1, checked_at=datetime.now(timezone.utc).isoformat(),
                  passed=False, failure_reasons=failures, battery=None, device_serial=None,
                  min_level_percent=min_level if isinstance(min_level, (int, float)) and math.isfinite(min_level) else None,
                  min_level_policy=THRESHOLD_NOTE,
                  read_only=True, app_started=False, recording_started=False,
                  phone_settings_changed=False, robot_commands_sent=0)
    if isinstance(min_level, bool) or not isinstance(min_level, (int, float)) or not math.isfinite(min_level) or not 0 <= min_level <= 100:
        failures.append('INVALID_MIN_LEVEL: 0〜100の有限な運用閾値を指定してください')
        return result
    executable = str(adb or shutil.which('adb') or
                     Path(os.environ.get('LOCALAPPDATA', '')) / 'Android/Sdk/platform-tools/adb.exe')
    result['adb'] = executable
    try:
        inventory = _adb_read([executable, 'devices'])
        if inventory.returncode:
            raise RuntimeError('ADB_DEVICES_FAILED: ' + (inventory.stderr or inventory.stdout).strip())
        devices = parse_adb_devices(inventory.stdout)
        result['devices'] = devices
        selected = select_device(devices, serial)
        result['device_serial'] = selected
        measured = _adb_read([executable, '-s', selected, 'shell', 'dumpsys', 'battery'])
        if measured.returncode:
            raise RuntimeError('ADB_BATTERY_READ_FAILED: ' + (measured.stderr or measured.stdout).strip())
        battery = parse_battery(measured.stdout)
        result['battery'] = battery
        if battery['level_percent'] < min_level:
            failures.append('BATTERY_BELOW_OPERATIONAL_MINIMUM')
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        failures.append(str(error))
    result['passed'] = not failures
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adb')
    parser.add_argument('--serial')
    parser.add_argument('--min-level', type=float, default=20, help='0〜100%%。既定20%%は可変の運用基準であり物理性能の保証ではない')
    parser.add_argument('--output', type=Path, help='新規JSON保存先。既存ファイルは上書きしない')
    args = parser.parse_args()
    result = inspect_camera_health(args.adb, args.serial, args.min_level)
    if args.output:
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
