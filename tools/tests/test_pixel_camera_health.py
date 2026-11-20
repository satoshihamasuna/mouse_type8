"""Androidの典型出力・非100scale・残量不明・端末選択を実測前提で検証する。"""
from pathlib import Path
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pixel_camera_health as health


ANDROID_USB = '''Current Battery Service state:
  AC powered: false
  USB powered: true
  Wireless powered: false
  Max charging current: 500000
  Max charging voltage: 5000000
  Charge counter: 913000
  status: 2
  health: 2
  present: true
  level: 25
  scale: 100
  voltage: 3826
  temperature: 299
  technology: Li-ion
'''
ONE_DEVICE = 'List of devices attached\nPIXEL123\tdevice product:bramble model:Pixel_4a_5G\n'


class BatteryParserTests(unittest.TestCase):
    def test_typical_android_usb_charging_output_and_units(self):
        result = health.parse_battery(ANDROID_USB)
        self.assertEqual(result['level'], 25)
        self.assertEqual(result['scale'], 100)
        self.assertEqual(result['level_percent'], 25.)
        self.assertEqual(result['status_name'], 'charging')
        self.assertEqual(result['plugged'], 2)
        self.assertEqual(result['power_sources'], ['usb'])
        self.assertEqual(result['voltage_mv'], 3826)
        self.assertAlmostEqual(result['temperature_c'], 29.9)

    def test_non100_scale_is_converted_before_threshold_comparison(self):
        output = ANDROID_USB.replace('level: 25', 'level: 51').replace('scale: 100', 'scale: 200')
        self.assertEqual(health.parse_battery(output)['level_percent'], 25.5)

    def test_numeric_plugged_mask_and_multiple_power_sources(self):
        output = '\n'.join(line for line in ANDROID_USB.splitlines() if 'powered:' not in line)
        result = health.parse_battery(output+'\n  plugged: 5\n')
        self.assertEqual(result['power_sources'], ['ac', 'wireless'])

    def test_missing_or_unknown_required_values_do_not_become_zero(self):
        for key in ('level', 'scale', 'status', 'voltage', 'temperature', 'present'):
            with self.subTest(key=key):
                output = '\n'.join(line for line in ANDROID_USB.splitlines() if not line.strip().startswith(key+':'))
                with self.assertRaises(ValueError):
                    health.parse_battery(output)
        with self.assertRaises(ValueError):
            health.parse_battery(ANDROID_USB.replace('temperature: 299', 'temperature: unknown'))

    def test_unknown_status_battery_absence_and_frozen_test_readings_are_rejected(self):
        for output in (ANDROID_USB.replace('status: 2', 'status: 1'),
                       ANDROID_USB.replace('status: 2', 'status: 9'),
                       ANDROID_USB.replace('present: true', 'present: false'),
                       'UPDATES STOPPED -- use reset to restart\n'+ANDROID_USB,
                       ANDROID_USB+'\nmUpdatesStopped=true'):
            with self.subTest(output=output):
                with self.assertRaises(ValueError):
                    health.parse_battery(output)

    def test_invalid_scale_level_voltage_and_power_evidence_are_rejected(self):
        for before, after in (('scale: 100','scale: 0'), ('scale: 100','scale: -100'),
                              ('level: 25','level: 101'), ('level: 25','level: -1'),
                              ('voltage: 3826','voltage: 0'), ('USB powered: true','USB powered: unknown')):
            with self.subTest(after=after):
                with self.assertRaises(ValueError):
                    health.parse_battery(ANDROID_USB.replace(before,after))
        with self.assertRaises(ValueError):
            health.parse_battery(ANDROID_USB+'\nplugged: 4\n')


class DeviceSelectionTests(unittest.TestCase):
    def test_single_device_uses_serial_not_inventory_order(self):
        devices = health.parse_adb_devices(ONE_DEVICE)
        self.assertEqual(health.select_device(devices), 'PIXEL123')

    def test_no_unknown_offline_unauthorized_or_multiple_device_is_implicitly_selected(self):
        for output in ('List of devices attached\n', 'List of devices attached\nP1\toffline\n',
                       'List of devices attached\nP1\tunauthorized\n',
                       'List of devices attached\nP1\tdevice\nP2\tdevice\n',
                       'List of devices attached\nP1\tdevice\nP2\toffline\n'):
            with self.subTest(output=output):
                with self.assertRaises(ValueError):
                    health.select_device(health.parse_adb_devices(output))

    def test_explicit_serial_selects_exact_authorized_device(self):
        devices = health.parse_adb_devices('List of devices attached\nP1\tdevice\nP2\tunauthorized\n')
        self.assertEqual(health.select_device(devices, 'P1'), 'P1')
        for serial in ('P', 'P2', 'missing'):
            with self.assertRaises(ValueError):
                health.select_device(devices, serial)

    def test_ambiguous_duplicate_serial_is_rejected(self):
        with self.assertRaises(ValueError):
            health.parse_adb_devices('List of devices attached\nP1\tdevice\nP1\tdevice\n')


class InspectionTests(unittest.TestCase):
    def inspect(self, output=ANDROID_USB, *, devices=ONE_DEVICE, min_level=20, serial=None):
        calls = []
        def adb(command, **kwargs):
            calls.append(command)
            self.assertNotIn('shell', kwargs)
            if command == ['adb-fixture', 'devices']:
                return subprocess.CompletedProcess(command, 0, devices, '')
            self.assertEqual(command, ['adb-fixture', '-s', 'PIXEL123', 'shell', 'dumpsys', 'battery'])
            return subprocess.CompletedProcess(command, 0, output, '')
        with patch.object(health.subprocess, 'run', side_effect=adb):
            result = health.inspect_camera_health('adb-fixture', serial, min_level)
        json.dumps(result, allow_nan=False)
        self.assertTrue(result['read_only'])
        self.assertFalse(result['app_started'])
        self.assertFalse(result['recording_started'])
        self.assertFalse(result['phone_settings_changed'])
        self.assertEqual(result['robot_commands_sent'], 0)
        return result, calls

    def test_only_devices_and_selected_dumpsys_are_executed(self):
        result, calls = self.inspect()
        self.assertTrue(result['passed'])
        self.assertEqual(len(calls), 2)
        self.assertEqual(result['device_serial'], 'PIXEL123')
        self.assertIn('運用', result['min_level_policy'])

    def test_low_charge_fails_even_when_usb_connected_and_charging(self):
        result, _ = self.inspect(ANDROID_USB.replace('level: 25','level: 19'))
        self.assertFalse(result['passed'])
        self.assertIn('BATTERY_BELOW_OPERATIONAL_MINIMUM', result['failure_reasons'])
        self.assertEqual(result['battery']['plugged'], 2)

    def test_threshold_uses_percentage_and_is_configurable(self):
        output = ANDROID_USB.replace('level: 25', 'level: 30').replace('scale: 100', 'scale: 200')
        self.assertFalse(self.inspect(output)[0]['passed'])
        self.assertTrue(self.inspect(output, min_level=15)[0]['passed'])
        self.assertTrue(self.inspect(ANDROID_USB.replace('level: 25','level: 20'))[0]['passed'])

    def test_unknown_battery_or_multiple_devices_stops_with_failure(self):
        result, _ = self.inspect('Battery service unavailable')
        self.assertFalse(result['passed'])
        self.assertIsNone(result['battery'])
        result, calls = self.inspect(devices='List of devices attached\nP1\tdevice\nP2\tdevice\n')
        self.assertFalse(result['passed'])
        self.assertEqual(len(calls), 1)

    def test_invalid_thresholds_are_json_serializable_failures_without_adb(self):
        with patch.object(health.subprocess, 'run') as run:
            for value in (float('nan'), float('inf'), -1, 101, True, '20'):
                result = health.inspect_camera_health('unused', min_level=value)
                self.assertFalse(result['passed'])
                json.dumps(result, allow_nan=False)
            run.assert_not_called()

    def test_adb_failure_and_timeout_are_not_success(self):
        for response in (FileNotFoundError('adb unavailable'), subprocess.TimeoutExpired('adb',10)):
            with patch.object(health.subprocess, 'run', side_effect=response):
                result = health.inspect_camera_health('missing-adb')
            self.assertFalse(result['passed'])
            self.assertIsNone(result['battery'])
        with patch.object(health.subprocess, 'run', return_value=subprocess.CompletedProcess('adb',1,'','device disconnected')):
            self.assertFalse(health.inspect_camera_health('adb')['passed'])


if __name__ == '__main__':
    unittest.main()
