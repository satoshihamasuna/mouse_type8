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


def valid_pillar():
    data=diag.empty_pillar_diagnostics()
    data['summary']=dict(schema='1',detector='1',applied='0',total='1',straight='1',diagonal='0',quality_rejected='0',stored='1',dropped='0')
    event=dict(index='0',ms='1010',minimum_ms='998',axis='0',side='1',motion='19',control='1',turn='0',
               feature='straight_pillar_adc_maximum',mode='shadow',reason='shadow',flags='0',generation='3',event_generation='3',
               feature_mm='39.5',delay_mm='5',sample_delay_mm='5',width_mm='0',prominence='200',before_mm='44.5',after_mm='44.5',
               delta_mm='0',reference_valid='1',proposal_valid='1',reference_mm='39.5',target_mm='44.5',innovation_mm='0',
               proposed_delta_mm='0',coordinate_jump_mm='0',minimum_length_mm='39.5',e_mm='4',psi_rad='.02',gyro_radps='.1',
               control_ir='0',control_ir_dot='0',confirm_e_mm='4',confirm_psi_rad='.02',confirm_gyro_radps='.1',
               sl_mm='80',sr_mm='80',fl_mm='125',fr_mm='125',confirm_fl_mm='125',confirm_fr_mm='125')
    for sensor in ('l','r','fl','fr'):event['adc_'+sensor]='200';event['range_'+sensor]='2'
    data['events']=[event]
    return data


class WallDiagnosticsTests(unittest.TestCase):
    def test_diagonal_framework_uncalibrated_quality_and_ready_are_shadow_only(self):
        data=valid_pillar();e=data['events'][0]
        data['summary'].update(straight='0',diagonal='1')
        e.update(axis='1',feature='diagonal_pillar_minimum',reference_valid='0',proposal_valid='0',
                 mode='control',reason='invalid',position_evaluated='1',position_phase='6',position_reason='invalid',
                 position_shadow_only='1',position_model_ready='0',position_status='uncalibrated',
                 model_reference_mm='0',model_pose_shift_mm='0',model_e_mm='4',model_psi_rad='.02',position_quality_flags='0')
        self.assertTrue(diag.validate_pillar(data))
        e.update(flags='8',reason='quality_rejected',position_status='quality_rejected',position_quality_flags='8')
        data['summary']['quality_rejected']='1';self.assertTrue(diag.validate_pillar(data))
        e.update(flags='0',reason='shadow',position_reason='shadow',position_status='ready',position_quality_flags='0',
                 position_model_ready='1',reference_valid='1',proposal_valid='1',proposed_delta_mm='1',
                 ekf='1',ekf_nis='.5',ekf_gain='.4',ekf_variance_mm2='8')
        data['summary']['quality_rejected']='0';self.assertTrue(diag.validate_pillar(data))
        self.assertEqual(diag.decoded_row(e)['position_phase_name'],'diagonal_run')
        e.update(reason='accepted',position_reason='accepted');self.assertFalse(diag.validate_pillar(data))
        e.update(reason='shadow',position_reason='shadow',delta_mm='1',after_mm='45.5');self.assertFalse(diag.validate_pillar(data))

    def test_diagonal_wall_framework_rejects_actual_application_and_invalid_metadata(self):
        data=valid();e=data['events'][0];data['summary'].update(accepted='0',invalid='1')
        e.update(phase='6',reason='invalid',delta_mm='0',after_mm=e['before_mm'],position_shadow_only='1',
                 position_model_ready='0',position_status='uncalibrated',model_reference_mm='0',
                 model_pose_shift_mm='0',model_e_mm='2',model_psi_rad='.01')
        self.assertTrue(diag.validate(data))
        e['position_shadow_only']='0';self.assertFalse(diag.validate(data));e['position_shadow_only']='1'
        e['position_status']='unknown';self.assertFalse(diag.validate(data))
        e['position_status']='uncalibrated';e['delta_mm']='1';self.assertFalse(diag.validate(data))
        e['delta_mm']='0';e['phase']='4';self.assertFalse(diag.validate(data))

    def test_pillar_entry_allows_negative_reference_and_explicit_minimum_rebase(self):
        data=valid_pillar();event=data['events'][0];data['summary']['applied']='1'
        event.update(mode='control',reason='accepted',position_evaluated='1',position_reason='accepted',
                     position_phase='0',minimum_source_generation='2',minimum_rebase_mm='-90',
                     generation='3',event_generation='3',minimum_length_mm='-2',feature_mm='-1',
                     reference_mm='-3',delay_mm='13',sample_delay_mm='14',before_mm='12',
                     target_mm='10',innovation_mm='-2',delta_mm='-1',after_mm='11',
                     ekf='1',ekf_nis='.2',ekf_gain='.5',ekf_variance_mm2='8')
        self.assertTrue(diag.validate_pillar(data))
        event['position_phase']='2';self.assertFalse(diag.validate_pillar(data))
        event['position_phase']='5';self.assertTrue(diag.validate_pillar(data))

    def test_pillar_ekf_application_exports_delta_and_checks_mode_counts_and_gain(self):
        data=valid_pillar();event=data['events'][0]
        data['summary']['applied']='1'
        event.update(mode='control',reason='accepted',position_evaluated='1',position_reason='accepted',
                     ekf='1',ekf_nis='.16',ekf_gain='.5',ekf_variance_mm2='8',
                     after_mm='43.5',delta_mm='-1',target_mm='42.5',innovation_mm='-2')
        self.assertTrue(diag.validate_pillar(data))
        for key,value in [('mode','shadow'),('ekf_gain','.8'),('after_mm','44.5'),('flags','1')]:
            old=event[key];event[key]=value;self.assertFalse(diag.validate_pillar(data));event[key]=old
        data['summary']['applied']='0';self.assertFalse(diag.validate_pillar(data))

    def test_pillar_shadow_exports_peak_time_pose_and_front_observations(self):
        data=valid_pillar();self.assertTrue(diag.validate_pillar(data))
        sync=dict(mcu_origin_ms=1000,video_time_at_mcu_origin_s=2.,clock_scale=1.,accepted=True)
        rr=review_rows(dict(pillar_diagnostics=data),sync)
        self.assertAlmostEqual(rr[0]['video_s'],1.998)
        self.assertAlmostEqual(rr[0]['confirmation_video_s'],2.010)
        self.assertEqual(rr[0]['e_mm'],'4')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'run.json';diag.export_tables(p,dict(pillar_diagnostics=data))
            with p.with_suffix('.pillar_candidates.csv').open(encoding='utf-8-sig',newline='') as f:r=list(csv.DictReader(f))
            self.assertEqual(r[0]['range_l_name'],'far_limit')
            self.assertEqual(r[0]['minimum_ms'],'998')

    def test_pillar_missing_corrupt_or_applied_records_are_incomplete(self):
        for change in (lambda d:d.update(events=[]),lambda d:d['events'][0].update(delta_mm='1'),
                       lambda d:d['events'][0].update(minimum_ms='1011'),lambda d:d['events'][0].update(flags='1'),
                       lambda d:d['events'][0].update(feature='wall_edge_crossing'),lambda d:d['events'][0].update(psi_rad='nan'),
                       lambda d:d['events'][0].update(mode='legacy'),lambda d:d['summary'].update(quality_rejected='1')):
            data=valid_pillar();change(data);self.assertFalse(diag.validate_pillar(data))
        data=valid_pillar()
        for i in range(70):diag.receive_pillar(data,'PILLAR_EDGE',dict(data['events'][0],index=str(i)))
        self.assertEqual(len(data['events']),64);self.assertFalse(diag.validate_pillar(data))
        data=valid_pillar();data['summary']['quality_rejected']='1'
        data['events'][0].update(flags='64',reason='quality_rejected',reference_valid='0',proposal_valid='0',psi_rad='nan')
        self.assertTrue(diag.validate_pillar(data))

    def test_trial_shadow_and_rejected_candidates_keep_proposal_without_applying_it(self):
        for reason in ('shadow', 'not_selected', 'stale_generation'):
            data = valid()
            data['summary'].update(accepted='0', **{reason: '1'})
            event = data['events'][0]
            event.update(reason=reason, mode='shadow', proposed_reason='limited',
                         proposed_delta_mm='-1', proposed_after_mm='2',
                         delta_mm='0', after_mm=event['before_mm'], generation='3',
                         event_generation='2' if reason == 'stale_generation' else '3',
                         feature='diagonal_difference_crossing')
            self.assertTrue(diag.validate(data))
            event['delta_mm'] = '-1'
            self.assertFalse(diag.validate(data))

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
            data['events'][0]['phase']='5'
            self.assertTrue(diag.validate(data))
            diag.export_tables(path,settings)
            self.assertEqual(original,path.read_bytes())
            with path.with_suffix('.wall_corrections.csv').open(encoding='utf-8-sig',newline='') as stream:
                self.assertEqual(next(csv.DictReader(stream))['phase_name'],'straight_run')

    def test_missing_duplicate_reordered_and_corrupt_diagnostics_stay_incomplete(self):
        for change in (lambda d: d.update(events=[]),
                       lambda d: d['events'].append(d['events'][0]),
                       lambda d: d['events'][0].update(index='1'),
                       lambda d: d['events'][0].update(delta_mm='nan'),
                       lambda d: d['events'][0].update(range_l='9'),
                       lambda d: d['summary'].update(total='2'),
                       lambda d: d['summary'].update(accepted='0', shadow='1'),
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
