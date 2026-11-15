"""B右V90の接続・候補復元・静止照合を連続実行する。走行指令は送らない。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
CAMERA_PYTHON = Path('C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe')
ANALYSIS_PYTHON = Path('D:/venv/venv3-11/Scripts/python.exe')
CLI = ROOT/'tools/pixel_camera_tuning_cli.py'


def load(path):
    for attempt, delay in enumerate((.05, .1, .2, 0)):
        try:
            return json.loads(Path(path).read_text(encoding='utf-8'))
        except PermissionError:
            if attempt == 3:
                raise
            time.sleep(delay)


def save(path, value, exclusive=False):
    with Path(path).open('x' if exclusive else 'w', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def validate_files(meta):
    assert meta['motion'] == 'r_v90' and meta['preset'] == 500
    assert meta['execution_commands_sent'] == meta['execution_resends'] == 0
    assert meta['offline_preflight_helper_check_passed']
    plan = load(meta['plan'])
    source_path = Path(plan['A_reference']['source'])
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == plan['A_reference']['sha256']
    reference = load(source_path)
    assert reference['mouse'] == 'A' and reference['motion'] == 'r_v90'
    assert reference['actual_settings']['parameters'] == plan['initial_geometry_from_A']
    validation = load(meta['offline_reference_check'])
    assert validation['passed'] is True
    for path, expected in validation['input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    for key in ('source_A', 'source_B'):
        assert hashlib.sha256((ROOT/plan[key]['path']).read_bytes()).hexdigest() == plan[key]['sha256']
    if meta.get('camera_static_validation'):
        validation = load(meta['camera_static_validation'])
        assert validation['state'] == 'PASSED_INDEPENDENT_STATIC_VALIDATION'
        for path, expected in validation['input_sha256'].items():
            assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    for item in plan['B_control_reference']:
        path = Path(item['path'])
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']
        evidence = load(path)
        assert evidence['mouse'] == 'B' and evidence['pid'] == plan['expected_candidate_pid']
    assert plan['start_placement_status'] == 'REGISTERED_STATIC_REFERENCE_REPOSITION_CONFIRMATION_REQUIRED'
    for field in ('start_placement_registry', 'position_confirmation_report'):
        assert hashlib.sha256(Path(plan[field]).read_bytes()).hexdigest() == plan[field+'_sha256']
    off = load(plan['position_confirmation_report'])
    assert off['state'] == 'POSITION_CONFIRMED_UNCHANGED_AND_OFF_BY_OPERATOR'
    assert off['operator_position_maintained_and_power_off'] is True
    assert all(item['ready'] for item in off['frames'])
    for field in ('baseline_actual_settings', 'baseline_final_validity'):
        assert hashlib.sha256(Path(plan[field]).read_bytes()).hexdigest() == plan[field+'_sha256']
    previous = load(plan['baseline_actual_settings'])
    assert previous['mouse'] == 'B' and previous['motion_name'] == 'r_v90'
    assert previous['pid'] == plan['expected_candidate_pid']
    assert {key:[value,plan['expected_candidate_parameters'][key]] for key,value in previous['parameters'].items() if value != plan['expected_candidate_parameters'][key]} == plan['changes_from_last_trial']
    validity = load(plan['baseline_final_validity'])
    assert validity['full_trajectory_review_passed'] and validity['dense_static_confirmation_passed']
    return reference, plan


def validate_readiness(meta):
    assert meta['operator_ready'] == 'USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON'
    ready = load(Path(meta['session'])/'operator_ready.json')
    assert ready['placement_stopped_clearance_hands_off_both_on_confirmed'] is True
    assert ready['capture_request_id'] == meta['capture_request_id']
    age = (datetime.now(timezone.utc)-datetime.fromisoformat(ready['recorded_at'])).total_seconds()
    assert 0 <= age < 300, '今回の配置・手離し確認が必要'
    assert ready.get('physical_hole_full_contact_clearance_confirmed') is True, '全接地点と全経路の穴・全幅・停止余裕の今回の確認が必要'
    assert ready.get('placement_matches_registered_static_picture_confirmed') is True, '登録静止画像と今回の配置の照合が必要'


def check_parameters(actual, expected):
    import math
    assert actual.keys() == expected.keys()
    # shell_debug_turn_show casts float*1000 to long (truncation).
    # Permit one display LSB, not a physical tuning tolerance.
    for key, value in expected.items():
        assert math.isfinite(actual[key]) and abs(actual[key]-value) <= .0010000001, key


def check_baseline(actual, plan):
    assert actual['mouse'] == 'B' and actual['motion_name'] == 'r_v90'
    assert actual['preset_speed_mm_s'] == 500 and actual['turn_count'] == 1
    check_parameters(actual['parameters'], plan['expected_compiled_B'])
    assert actual['pid'] == plan['expected_B_pid']
    assert not actual['suction']['enabled']
    import math
    assert actual['feedforward'] and all(math.isfinite(v) for v in actual['feedforward'].values())


def build_candidate(baseline, plan):
    # Never use A or another B turn PID/FF: retain the fresh complete B baseline.
    check_baseline(baseline, plan)
    from copy import deepcopy
    source = deepcopy(baseline)
    source['parameters'] = deepcopy(plan['expected_candidate_parameters'])
    source['pid'] = deepcopy(plan['expected_candidate_pid'])
    assert {key:[value,source['pid'][key]] for key,value in baseline['pid'].items() if value != source['pid'][key]} == plan['allowed_B_pid_changes']
    source['source'] = 'unapplied_B_geometry_comparison_preserving_fresh_B_FF'
    source.pop('raw_response', None)
    source.pop('log', None)
    return source


def check_candidate(actual, source):
    for key in ('mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count',
                'pid', 'feedforward', 'suction'):
        assert actual[key] == source[key], key
    check_parameters(actual['parameters'], source['parameters'])


def prepare(meta_path):
    meta = load(meta_path)
    reference, plan = validate_files(meta)
    validate_readiness(meta)
    session = Path(meta['session'])
    assert not (session/'service.json').exists()
    assert not (session/'jobs'/f"{meta['capture_request_id']}.json").exists()
    started = time.monotonic()
    timings = []
    save(session/'fast_prepare_started.json', dict(at=datetime.now(timezone.utc).isoformat(),
         meta=str(meta_path), motion_commands_sent=0), exclusive=True)
    env = dict(os.environ)
    for key in ('OPENCV_FOR_THREADS_NUM', 'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        env[key] = '1'

    def stage(name, command, timeout):
        before = time.monotonic()
        completed = subprocess.run([str(item) for item in command], cwd=ROOT, env=env,
            capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=timeout)
        (session/f'fast_{name}.stdout.log').write_text(completed.stdout, encoding='utf-8')
        (session/f'fast_{name}.stderr.log').write_text(completed.stderr, encoding='utf-8')
        timings.append(dict(stage=name, seconds=time.monotonic()-before, returncode=completed.returncode))
        print(json.dumps(timings[-1]), flush=True)
        assert completed.returncode == 0, f'{name}失敗。ログを保持し、走行せず停止'

    stdout = (session/'fast_service.stdout.log').open('x', encoding='utf-8')
    stderr = (session/'fast_service.stderr.log').open('x', encoding='utf-8')
    try:
        process = subprocess.Popen([str(CAMERA_PYTHON), str(CLI), '--session', str(session),
            'serve', '--config', meta['config']], cwd=ROOT, env=env, stdout=stdout, stderr=stderr,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    finally:
        stdout.close()
        stderr.close()
    meta['service_pid'] = process.pid
    save(meta_path, meta)
    deadline = time.monotonic()+25
    while not (session/'service.json').exists():
        assert process.poll() is None, '接続サービスが起動に失敗。再接続しない'
        assert time.monotonic() < deadline, '接続待機時間超過。再接続しない'
        time.sleep(.2)
    timings.append(dict(stage='connect', seconds=time.monotonic()-started, returncode=0))
    print(json.dumps(timings[-1]), flush=True)
    prefix = [CAMERA_PYTHON, CLI, '--session', session]
    identifier = meta['capture_request_id']
    stage('preset', prefix+['preset', '--motion', 'r_v90', '--preset', '500',
        '--output', meta['preset_readback'], '--request-id', identifier+'_preset'], 45)
    baseline = load(meta['preset_readback'])
    check_baseline(baseline, plan)
    assert baseline['feedforward'] == load(plan['baseline_actual_settings'])['feedforward']
    assert baseline['suction'] == load(plan['baseline_actual_settings'])['suction']
    source = build_candidate(baseline, plan)
    save(Path(meta['settings_source']), source, exclusive=True)
    stage('apply', prefix+['apply', '--settings', meta['settings_source'],
        '--output', meta['settings'], '--request-id', identifier+'_apply'], 60)
    check_candidate(load(meta['settings']), source)
    helper = Path(meta['preflight_helper'])
    stage('static_record', [CAMERA_PYTHON, '-X', 'utf8', helper, 'record', '--session', session], 40)
    stage('static_review', [ANALYSIS_PYTHON, '-X', 'utf8', helper, 'review', '--session', session], 40)
    checked = load(session/'static_preflight_check.json')
    assert checked['ready'] is True
    for path, expected in checked['input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    meta.update(state='PREFLIGHT_PASSED_CAPTURE_READY', preflight_ready=True)
    save(meta_path, meta)
    ready_snapshot = session/'fast_prepare_metadata_ready_snapshot.json'
    save(ready_snapshot, meta, exclusive=True)
    result = dict(state='READY_FOR_SEPARATE_CAPTURE', timings=timings,
        total_seconds=time.monotonic()-started, motion_commands_sent=0,
        execution_request_not_submitted=True, input_sha256={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest()
            for path in (Path(__file__), ready_snapshot, helper, Path(meta['settings']), session/'static_preflight_check.json')})
    save(session/'fast_prepare_result.json', result, exclusive=True)
    print(json.dumps(dict(state=result['state'], seconds=result['total_seconds'], motion_commands_sent=0)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--meta', type=Path, required=True)
    parser.add_argument('--offline-check', action='store_true')
    args = parser.parse_args()
    if args.offline_check:
        validate_files(load(args.meta))
        print('OFFLINE_FILES_PASSED; 機器接続・設定変更・走行指令なし')
    else:
        prepare(args.meta.resolve())


if __name__ == '__main__':
    main()
