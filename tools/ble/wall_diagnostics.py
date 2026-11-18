"""走行後の壁診断。既存55列テレメトリとは独立したschema 1。"""
import csv
import math
from collections import Counter


RANGE_NAMES = ('in_range', 'near_limit', 'far_limit', 'invalid', 'unknown')
PHASE_NAMES = ('straight_entry', 'diagonal_entry', 'straight_exit', 'diagonal_exit', 'search_straight', 'straight_run', 'diagonal_run')
DIAGONAL_MODEL_STATUSES = ('ready','uncalibrated','quality_rejected','pose_outside','duplicate','wrong_side','stale_generation','invalid','outside_window','nis_rejected')
OPTIONAL_DECISIONS = ('shadow', 'not_selected', 'stale_generation')
REASONS = ('accepted', 'limited', 'outside_window', 'invalid', 'wrong_side') + OPTIONAL_DECISIONS
PILLAR_FLAGS = ('rotating_window', 'frame_changed', 'position_jump', 'front_pair_close', 'side_near_limit', 'invalid_front', 'invalid_pose')
CHECK_NAMES = ('metric_pair', 'pair_window', 'centered', 'lateral_certain', 'aligned',
               'heading_certain', 'quiet_yaw', 'pose_trusted', 'ekf_control', 'pose_used')


def empty_diagnostics():
    return {'range_summary': None, 'summary': None, 'events': [], 'complete': False, 'errors': []}


def empty_pillar_diagnostics():
    return {'summary': None, 'events': [], 'complete': False, 'errors': []}


def receive_pillar(diagnostics, tag, values):
    if tag == 'PILLAR_EDGE':
        if len(diagnostics['events']) < 64:
            diagnostics['events'].append(values)
        elif 'event_overflow' not in diagnostics['errors']:
            diagnostics['errors'].append('event_overflow')
    elif tag == 'PILLAR_STATS':
        if diagnostics['summary'] is not None and 'duplicate_summary' not in diagnostics['errors']:
            diagnostics['errors'].append('duplicate_summary')
        diagnostics['summary'] = values


def _validate_diagonal_model(e, phase):
    if e.get('position_shadow_only','0') == '0':
        return False
    _require(e['position_shadow_only'] == '1' and phase in (1,3,6))
    _require(e['position_model_ready'] in ('0','1') and e['position_status'] in DIAGONAL_MODEL_STATUSES)
    _require(float(e['delta_mm']) == 0 and float(e['after_mm']) == float(e['before_mm']))
    _require(e.get('position_reason',e['reason']) not in ('accepted','limited'))
    if 'position_quality_flags' in e:_require(0 <= int(e['position_quality_flags']) < 128)
    for key in ('model_reference_mm','model_pose_shift_mm','model_e_mm','model_psi_rad'):
        _require(math.isfinite(float(e[key])) or bool(int(e.get('flags','0')) & 64) or e['position_status']=='invalid')
    if e['position_status']=='ready':
        _require(e['position_model_ready']=='1' and e.get('position_reason',e['reason'])=='shadow')
        _require(abs(float(e['proposed_delta_mm'])) <= 3.001)
    if e.get('ekf')=='1':
        _require(all(math.isfinite(float(e[k])) for k in ('ekf_nis','ekf_gain','ekf_variance_mm2')))
        _require(0 <= float(e['ekf_gain']) <= .50001 and float(e['ekf_variance_mm2']) >= 0)
    if e['position_model_ready']=='0':
        _require(e.get('reference_valid','0')=='0' and e.get('proposal_valid','0')=='0')
    return True


def _validate_diagonal_pose(event, axis):
    if not any(key.startswith('diag_pose_') for key in event):
        return
    _require(axis == 1 and event['position_shadow_only'] == '1')
    status = int(event['diag_pose_status'])
    pillar = int(event['diag_pose_pillar'])
    used = int(event['diag_pose_used'])
    _require(1 <= status <= 6 and 0 <= pillar <= 128 and 0 <= used <= 128)
    _require((pillar > 0) == (status in (5, 6)) and (status != 6 or used > 0))
    _require(event['diag_pose_ready'] in ('0', '1') and (event['diag_pose_ready'] == '0' or used >= 3))
    for suffix in ('s_mm', 'e_mm', 'heading_rad', 'innovation_mm'):
        _require(math.isfinite(float(event['diag_pose_' + suffix])))
    for suffix in ('sigma_s_mm', 'sigma_e_mm', 'sigma_heading_rad', 'nis'):
        value = float(event['diag_pose_' + suffix])
        _require(math.isfinite(value) and value >= 0)


def validate_pillar(diagnostics):
    try:
        summary, events = diagnostics['summary'], diagnostics['events']
        _require(summary['schema'] == '1' and summary['detector'] == '1')
        stored, total, dropped = (int(summary[k]) for k in ('stored', 'total', 'dropped'))
        _require(0 <= stored <= 64 and total >= stored and dropped == total-stored)
        straight, diagonal, rejected = (int(summary[k]) for k in ('straight', 'diagonal', 'quality_rejected'))
        _require(min(straight, diagonal, rejected) >= 0 and straight+diagonal == total and rejected <= total)
        applied = int(summary['applied']); _require(0 <= applied <= straight and applied <= total-rejected)
        _require([int(e['index']) for e in events] == list(range(stored)))
        observed = Counter(); previous = None
        for e in events:
            ms, minimum_ms = int(e['ms']), int(e['minimum_ms'])
            _require(0 <= ms <= 0xffffffff and 0 <= minimum_ms <= 0xffffffff and (ms-minimum_ms) % 2**32 < 2**31)
            if previous is not None:
                _require((ms-previous) % 2**32 < 2**31)
            previous = ms
            axis = int(e['axis']); _require(axis in (0, 1) and int(e['side']) in (1, 2))
            _validate_diagonal_pose(e, axis)
            _require(e['feature'] == ('straight_pillar_adc_maximum' if axis == 0 else 'diagonal_pillar_minimum'))
            _require(e['mode'] in ('shadow', 'control') and int(e['flags']) in range(128))
            _require(e['reason'] == ('quality_rejected' if int(e['flags']) else e.get('position_reason') if e.get('position_evaluated') == '1' else 'shadow'))
            observed['straight' if axis == 0 else 'diagonal'] += 1
            observed['quality_rejected'] += bool(int(e['flags']))
            for key in ('feature_mm', 'delay_mm', 'sample_delay_mm', 'width_mm', 'prominence', 'before_mm', 'after_mm',
                        'delta_mm', 'reference_mm', 'target_mm', 'innovation_mm', 'proposed_delta_mm', 'coordinate_jump_mm',
                        'minimum_length_mm', 'e_mm', 'psi_rad', 'gyro_radps', 'control_ir', 'control_ir_dot',
                        'confirm_e_mm', 'confirm_psi_rad', 'confirm_gyro_radps', 'sl_mm', 'sr_mm', 'fl_mm', 'fr_mm', 'confirm_fl_mm', 'confirm_fr_mm'):
                _require(math.isfinite(float(e[key])) or bool(int(e['flags']) & 64))
            _require(all(math.isfinite(float(e[k])) and float(e[k]) >= 0 for k in ('delay_mm','sample_delay_mm','width_mm','prominence')))
            before, after = float(e['before_mm']),float(e['after_mm'])
            if e.get('position_evaluated') == '1':
                diagonal_shadow=_validate_diagonal_model(e,int(e.get('position_phase','4')))
                _require(e.get('position_reason') in REASONS)
                _require((axis==0 and not int(e['flags']) and not diagonal_shadow) or (axis==1 and diagonal_shadow))
                if 'position_phase' in e:_require(int(e['position_phase']) in ((1,3,6) if diagonal_shadow else (0,4,5)))
                if e['position_reason'] in ('accepted', 'limited'):
                    _require(axis==0 and e['mode'] == 'control' and e.get('ekf') == '1')
                    _require(abs(float(e['delta_mm'])) <= 4.001 and abs(after-before-float(e['delta_mm'])) <= .003)
                    observed['applied'] += 1
                else:
                    _require(float(e['delta_mm']) == 0 and after == before)
                if e.get('ekf') == '1':
                    _require(all(math.isfinite(float(e[k])) for k in ('ekf_nis','ekf_gain','ekf_variance_mm2')))
                    _require(0 <= float(e['ekf_gain']) <= .50001 and float(e['ekf_variance_mm2']) >= 0)
            else:
                _require(float(e['delta_mm']) == 0 and (after == before or (int(e['flags']) & 64 and math.isnan(after) and math.isnan(before))))
            _require(e['reference_valid'] in ('0', '1') and (e['reference_valid'] == '0' or (not int(e['flags']) and (axis==0 or e.get('position_shadow_only')=='1'))))
            _require(e['proposal_valid'] in ('0', '1') and (e['proposal_valid'] == '0' or e['reference_valid'] == '1'))
            for k in ('motion','control','turn'):
                _require(0 <= int(e[k]) <= 255)
            for k in ('generation','event_generation'):
                _require(0 <= int(e[k]) <= 0xffffffff)
            if 'minimum_source_generation' in e:
                _require(0 <= int(e['minimum_source_generation']) <= 0xffffffff)
                _require(math.isfinite(float(e['minimum_rebase_mm'])) or bool(int(e['flags']) & 64))
            for sensor in ('l','r','fl','fr'):
                _require(-32768 <= int(e['adc_'+sensor]) <= 32767 and 0 <= int(e['range_'+sensor]) < 5)
        for key, count in [('straight',straight),('diagonal',diagonal),('quality_rejected',rejected)]:
            _require(observed[key] <= count and (dropped > 0 or observed[key] == count))
        _require(observed['applied'] <= applied and (dropped > 0 or observed['applied'] == applied))
        _require(not diagnostics['errors'])
        diagnostics['complete'] = True
    except (KeyError, TypeError, ValueError):
        diagnostics['complete'] = False
    diagnostics['truncated'] = str((diagnostics.get('summary') or {}).get('dropped','0')).isdigit() and int((diagnostics.get('summary') or {}).get('dropped','0')) > 0
    return diagnostics['complete']


def receive(diagnostics, tag, values):
    """不正・重複応答は黙って成功にしない。保持件数は有界。"""
    if tag == 'WALL_CORR':
        if len(diagnostics['events']) < 64:
            diagnostics['events'].append(values)
        elif 'event_overflow' not in diagnostics['errors']:
            diagnostics['errors'].append('event_overflow')
    else:
        key = 'range_summary' if tag == 'WALL_RANGE_STATS' else 'summary'
        if diagnostics[key] is not None and 'duplicate_summary' not in diagnostics['errors']:
            diagnostics['errors'].append('duplicate_summary')
        diagnostics[key] = values


def _require(condition):
    if not condition:
        raise ValueError("inconsistent wall diagnostics")


def validate(diagnostics):
    try:
        summary, ranges, events = diagnostics['summary'], diagnostics['range_summary'], diagnostics['events']
        _require(summary['schema'] == ranges['schema'] == '1')
        stored, total, dropped = (int(summary[k]) for k in ('stored', 'total', 'dropped'))
        _require(0 <= stored <= 64 and total >= stored and dropped == total-stored)
        counts = [int(summary[k]) for k in ('accepted', 'limited', 'rejected', 'invalid', 'wrong_side')]
        counts += [int(summary.get(k, '0')) for k in OPTIONAL_DECISIONS]
        _require(sum(counts) == total)
        _require([int(e['index']) for e in events] == list(range(stored)))
        _require(all(c >= 0 for c in counts))
        previous = None
        observed = Counter()
        for e in events:
            ms = int(e['ms']); _require(0 <= ms <= 0xffffffff)
            if previous is not None: _require((ms-previous) % 2**32 < 2**31)
            previous = ms
            _require(e['reason'] in REASONS)
            observed['rejected' if e['reason'] == 'outside_window' else e['reason']] += 1
            _require(0 <= int(e['phase']) < len(PHASE_NAMES) and 1 <= int(e['side']) <= 3)
            diagonal_shadow=_validate_diagonal_model(e,int(e['phase']))
            _require(int(e['phase'])!=6 or diagonal_shadow)
            _require(0 <= int(e['motion']) <= 255)
            for key in ('before_mm', 'target_mm', 'delay_mm', 'innovation_mm', 'delta_mm', 'after_mm'):
                value = float(e[key])
                _require(math.isfinite(value) or e['reason'] == 'invalid')
            if 'mode' in e:
                _require(e['mode'] in ('legacy', 'shadow', 'control'))
                _require(e['proposed_reason'] in REASONS)
                for key in ('proposed_delta_mm', 'proposed_after_mm'):
                    _require(math.isfinite(float(e[key])) or e['reason'] == 'invalid')
                _require(0 <= int(e['generation']) <= 0xffffffff and 0 <= int(e['event_generation']) <= 0xffffffff)
                _require(e['feature'] in ('wall_edge_crossing', 'diagonal_difference_crossing'))
                if e['reason'] in OPTIONAL_DECISIONS + ('wrong_side', 'outside_window'):
                    _require(float(e['delta_mm']) == 0 and float(e['after_mm']) == float(e['before_mm']))
            for sensor in ('l', 'r', 'fl', 'fr'):
                _require(-32768 <= int(e['adc_'+sensor]) <= 32767)
                _require(0 <= int(e['range_'+sensor]) < len(RANGE_NAMES))
        for key in ('accepted', 'limited', 'rejected', 'invalid', 'wrong_side') + OPTIONAL_DECISIONS:
            count = int(summary.get(key, '0'))
            _require(observed[key] <= count and (dropped > 0 or observed[key] == count))
        moving, all_samples = int(ranges['moving_samples']), int(ranges['all_samples'])
        _require(0 <= moving <= all_samples)
        for sensor in ('l', 'r', 'fl', 'fr'):
            values = [int(ranges[sensor+'_'+s]) for s in ('ok', 'near', 'far', 'invalid')]
            _require(all(v >= 0 for v in values) and sum(values) == moving)
        _require(not diagnostics['errors'])
        diagnostics['complete'] = True
    except (KeyError, TypeError, ValueError):
        diagnostics['complete'] = False
    diagnostics['truncated'] = int((diagnostics.get('summary') or {}).get('dropped', '0')) > 0 if str((diagnostics.get('summary') or {}).get('dropped', '0')).isdigit() else False
    return diagnostics['complete']


def decoded_row(event):
    row = dict(event)
    if row.get('feature') in ('straight_pillar_adc_maximum','diagonal_pillar_minimum') and 'flags' in row:
        try: row['quality_flags'] = '|'.join(name for i,name in enumerate(PILLAR_FLAGS) if int(row['flags']) & (1 << i))
        except (ValueError,TypeError): row['quality_flags'] = ''
    for sensor in ('l', 'r', 'fl', 'fr'):
        try:
            index = int(event['range_'+sensor])
            row['range_'+sensor+'_name'] = RANGE_NAMES[index] if 0 <= index < 5 else 'unknown'
        except (KeyError, TypeError, ValueError):
            row['range_'+sensor+'_name'] = ''  # 旧ログから推測しない
    if 'phase' in row:
        try: row['phase_name'] = PHASE_NAMES[int(row['phase'])]
        except (IndexError, ValueError, TypeError): row['phase_name'] = ''
    if 'position_phase' in row:
        try: row['position_phase_name'] = PHASE_NAMES[int(row['position_phase'])]
        except (IndexError, ValueError, TypeError): row['position_phase_name'] = ''
    if 'checks' in row:
        try: row['passed_checks'] = '|'.join(name for i, name in enumerate(CHECK_NAMES) if int(row['checks']) & (1 << i))
        except (ValueError, TypeError): row['passed_checks'] = ''
    return row


def export_tables(settings_path, settings):
    for key, suffix in (('wall_diagnostics', '.wall_corrections.csv'), ('turn_diagnostics', '.turn_decisions.csv'), ('pillar_diagnostics', '.pillar_candidates.csv')):
        diagnostics = settings.get(key)
        if diagnostics is None:
            continue
        rows = [dict(decoded_row(e), diagnostics_complete=diagnostics.get('complete', False)) for e in diagnostics['events']]
        columns = list(dict.fromkeys(k for row in rows for k in row)) or ['index', 'ms', 'diagnostics_complete']
        path = settings_path.with_suffix(suffix)
        with path.open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader(); writer.writerows(rows)
