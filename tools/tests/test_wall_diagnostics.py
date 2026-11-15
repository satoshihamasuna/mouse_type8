"""壁診断の欠落検出、旧形式互換、動画時刻対応。通信・実機操作なし。"""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ble import wall_diagnostics as diag
from analysis.wall_event_review import review_rows


def valid():
    data = diag.empty_diagnostics()
    data['summary'] = dict(schema='1', total='1', accepted='1', limited='0', rejected='0',
                           invalid='0', wrong_side='0', stored='1', dropped='0')
    data['range_summary'] = dict(schema='1', all_samples='10', moving_samples='5')
    event = dict(index='0', ms='1000', motion='7', phase='1', side='2', reason='accepted',
                 before_mm='3', target_mm='1', delay_mm='1', innovation_mm='-2', delta_mm='-1', after_mm='2')
    for sensor in ('l', 'r', 'fl', 'fr'):
        event['adc_'+sensor] = '123'; event['range_'+sensor] = '0'
        for state in ('ok', 'near', 'far', 'invalid'):
            data['range_summary'][sensor+'_'+state] = '5' if state == 'ok' else '0'
    data['events'] = [event]
    return data


class WallDiagnosticsTests(unittest.TestCase):
    def test_valid_transfer_exports_units_states_and_does_not_modify_raw_json(self):
        data = valid(); self.assertTrue(diag.validate(data))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.json'
            settings = dict(wall_diagnostics=data)
            path.write_text(json.dumps(settings), encoding='utf-8')
            original = path.read_bytes()
            diag.export_tables(path, settings)
            self.assertEqual(original, path.read_bytes())
            with path.with_suffix('.wall_corrections.csv').open(encoding='utf-8-sig', newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]['range_l_name'], 'in_range')
            self.assertEqual(rows[0]['phase_name'], 'diagonal_entry')

    def test_missing_duplicate_reordered_and_corrupt_diagnostics_stay_incomplete(self):
        for change in (lambda d: d.update(events=[]),
                       lambda d: d['events'].append(d['events'][0]),
                       lambda d: d['events'][0].update(index='1'),
                       lambda d: d['events'][0].update(delta_mm='nan'),
                       lambda d: d['events'][0].update(range_l='9'),
                       lambda d: d['summary'].update(total='2'),
                       lambda d: d['range_summary'].update(l_ok='6'),
                       lambda d: d['summary'].update(schema='2')):
            data = valid(); change(data); self.assertFalse(diag.validate(data))

    def test_buffer_and_duplicate_headers_are_bounded_and_flagged(self):
        data = valid()
        for i in range(70): diag.receive(data, 'WALL_CORR', dict(data['events'][0], index=str(i)))
        self.assertEqual(len(data['events']),64)
        self.assertFalse(diag.validate(data))
        diag.receive(data, 'WALL_CORR_STATS', data['summary'])
        self.assertIn('duplicate_summary', data['errors'])

    def test_clock_scale_missing_old_fields_and_failed_sync_are_explicit(self):
        sync = dict(mcu_origin_ms=500, video_time_at_mcu_origin_s=2., clock_scale=.99, accepted=False)
        settings = dict(turn_diagnostics=dict(complete=True, events=[dict(index='2', ms='1000', reason='front_diff')]))
        rows = review_rows(settings, sync)
        self.assertAlmostEqual(rows[0]['video_s'], 2.495)
        self.assertEqual(rows[0]['range_l_name'], '')
        self.assertNotIn('adc_l', rows[0])
        self.assertEqual(review_rows({}, sync), [])
        self.assertEqual(diag.decoded_row(dict(checks='1023'))['passed_checks'].split('|'), list(diag.CHECK_NAMES))


if __name__ == '__main__': unittest.main()
