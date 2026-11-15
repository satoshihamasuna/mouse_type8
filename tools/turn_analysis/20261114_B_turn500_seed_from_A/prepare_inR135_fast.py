"""B右135入口の接続・候補復元・静止照合を連続実行する。走行指令は送らない。"""
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
    assert meta['motion'] == 'in_r135' and meta['preset'] == 500
    assert meta['execution_commands_sent'] == meta['execution_resends'] == 0
    assert meta['offline_preflight_helper_check_passed']
    source = load(meta['settings_source'])
    plan = load(meta['plan'])
    valid = load(meta['previous_final_validity'])
    assert valid['full_trajectory_review_passed'] and valid['dense_static_confirmation_passed']
    assert source['mouse'] == 'B' and source['motion_name'] == 'in_r135'
    assert source['preset_speed_mm_s'] == 500 and source['turn_count'] == 1
    assert source['parameters'] == plan['expected_candidate_parameters']
    assert source['pid'] == plan['expected_candidate_pid']
    assert not source['suction']['enabled']
    assert hashlib.sha256((ROOT/plan['source_B']['path']).read_bytes()).hexdigest() == plan['source_B']['sha256']
    if meta.get('camera_static_validation'):
        validation = load(meta['camera_static_validation'])
        assert validation['state'] == 'PASSED_INDEPENDENT_STATIC_VALIDATION'
        for path, expected in validation['input_sha256'].items():
            assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    return source, plan


def validate_readiness(meta):
    assert meta['operator_ready'] == 'USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON'
    ready = load(Path(meta['session'])/'operator_ready.json')
    assert ready['placement_stopped_clearance_hands_off_both_on_confirmed'] is True
    assert ready['capture_request_id'] == meta['capture_request_id']
    age = (datetime.now(timezone.utc)-datetime.fromisoformat(ready['recorded_at'])).total_seconds()
    assert 0 <= age < 300, '今回の配置・手離し確認が必要'


def check_baseline(actual, source, plan):
    assert actual['mouse'] == 'B' and actual['motion_name'] == 'in_r135'
    assert actual['preset_speed_mm_s'] == 500 and actual['turn_count'] == 1
    assert actual['parameters'] == plan['expected_compiled_B']
    assert actual['pid'] == plan['expected_B_pid']
    assert actual['feedforward'] == source['feedforward']
    assert actual['suction'] == source['suction']


def check_candidate(actual, source):
    for key in ('mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count',
                'parameters', 'pid', 'feedforward', 'suction'):
        assert actual[key] == source[key], key


def prepare(meta_path):
    meta = load(meta_path)
    source, plan = validate_files(meta)
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
    stage('preset', prefix+['preset', '--motion', 'in_r135', '--preset', '500',
        '--output', meta['preset_readback'], '--request-id', identifier+'_preset'], 45)
    check_baseline(load(meta['preset_readback']), source, plan)
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
