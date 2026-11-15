"""走行後の壁診断。既存55列テレメトリとは独立したschema 1。"""
import csv
import math


RANGE_NAMES = ('in_range', 'near_limit', 'far_limit', 'invalid', 'unknown')
PHASE_NAMES = ('straight_entry', 'diagonal_entry', 'straight_exit', 'diagonal_exit', 'search_straight')
CHECK_NAMES = ('metric_pair', 'pair_window', 'centered', 'lateral_certain', 'aligned',
               'heading_certain', 'quiet_yaw', 'pose_trusted', 'ekf_control', 'pose_used')


def empty_diagnostics():
    return {'range_summary': None, 'summary': None, 'events': [], 'complete': False, 'errors': []}


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
        _require(sum(int(summary[k]) for k in ('accepted', 'limited', 'rejected', 'invalid', 'wrong_side')) == total)
        _require([int(e['index']) for e in events] == list(range(stored)))
        counts = [int(summary[k]) for k in ('accepted', 'limited', 'rejected', 'invalid', 'wrong_side')]
        _require(all(c >= 0 for c in counts))
        previous = None
        for e in events:
            ms = int(e['ms']); _require(0 <= ms <= 0xffffffff)
            if previous is not None: _require((ms-previous) % 2**32 < 2**31)
            previous = ms
            _require(e['reason'] in ('accepted', 'limited', 'outside_window', 'invalid', 'wrong_side'))
            _require(0 <= int(e['phase']) < len(PHASE_NAMES) and 1 <= int(e['side']) <= 3)
            _require(0 <= int(e['motion']) <= 255)
            for key in ('before_mm', 'target_mm', 'delay_mm', 'innovation_mm', 'delta_mm', 'after_mm'):
                value = float(e[key])
                _require(math.isfinite(value) or e['reason'] == 'invalid')
            for sensor in ('l', 'r', 'fl', 'fr'):
                _require(-32768 <= int(e['adc_'+sensor]) <= 32767)
                _require(0 <= int(e['range_'+sensor]) < len(RANGE_NAMES))
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
    for sensor in ('l', 'r', 'fl', 'fr'):
        try:
            index = int(event['range_'+sensor])
            row['range_'+sensor+'_name'] = RANGE_NAMES[index] if 0 <= index < 5 else 'unknown'
        except (KeyError, TypeError, ValueError):
            row['range_'+sensor+'_name'] = ''  # 旧ログから推測しない
    if 'phase' in row:
        try: row['phase_name'] = PHASE_NAMES[int(row['phase'])]
        except (IndexError, ValueError, TypeError): row['phase_name'] = ''
    if 'checks' in row:
        try: row['passed_checks'] = '|'.join(name for i, name in enumerate(CHECK_NAMES) if int(row['checks']) & (1 << i))
        except (ValueError, TypeError): row['passed_checks'] = ''
    return row


def export_tables(settings_path, settings):
    for key, suffix in (('wall_diagnostics', '.wall_corrections.csv'), ('turn_diagnostics', '.turn_decisions.csv')):
        diagnostics = settings.get(key)
        if diagnostics is None:
            continue
        rows = [dict(decoded_row(e), diagnostics_complete=diagnostics.get('complete', False)) for e in diagnostics['events']]
        columns = list(dict.fromkeys(k for row in rows for k in row)) or ['index', 'ms', 'diagnostics_complete']
        path = settings_path.with_suffix(suffix)
        with path.open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader(); writer.writerows(rows)
