"""新規保存runを待ち、実機接続なしで追従・赤前点を評価する。"""
from pathlib import Path
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
LOCAL = BASE / 'baseline01/local_execution'
ANALYSIS = BASE / 'parallel_analysis'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def wait_for(predicate):
    while True:
        result = predicate()
        if result:
            return result
        time.sleep(2)


def saved_run():
    for run in sorted(LOCAL.glob('run_A_INR135_1800_baseline_20261120_*')):
        try:
            trial = read(run / 'trial.json')
            if trial['state'] == 'CAPTURED_REVIEW_REQUIRED' and trial.get('saved_log', {}).get('status') == 'saved':
                return run
        except (OSError, ValueError, KeyError):
            pass


def run_analysis(script, arguments, log):
    with log.open('x', encoding='utf-8') as output:
        result = subprocess.run([sys.executable, str(ROOT / 'tmp' / script), *map(str, arguments)],
            cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    print(json.dumps(dict(script=script, returncode=result.returncode, stdout=str(log))), flush=True)
    return result.returncode


print('Waiting for saved in_r135 baseline; hardware operations 0', flush=True)
run = wait_for(saved_run)
out = ANALYSIS / run.name.removeprefix('run_')
out.mkdir(parents=True, exist_ok=False)
print(json.dumps(dict(saved_run=str(run), output=str(out))), flush=True)
tracking_code = run_analysis('parallel_inr135_tracking_20261120.py', [run, out / 'tracking_1ms_v1'], out / 'tracking.stdout.txt')
report = wait_for(lambda: next((path for path in (run/'full/report.json', run/'quick/report.json') if path.is_file()), None))
red_code = run_analysis('parallel_inr135_red_reference_20261120.py',
    ['--run', run, '--out', out/'front_reference_v1', '--report', report], out/'front_reference.stdout.txt')
full = wait_for(lambda: run/'full/report.json' if (run/'full/report.json').is_file() else None)
body_quality = dict(source_report=str(full), required_motion_window_quality=read(full)['required_motion_window_quality'],
    summary_present=(run/'full/summary.json').is_file(), body_pair_quality_replaced_by_red=False,
    tracking_returncode=tracking_code, front_returncode=red_code, hardware_operations=0, parameter_adoption_allowed=False)
with (out/'body_pair_full_quality_preserved.json').open('x', encoding='utf-8') as file:
    json.dump(body_quality, file, ensure_ascii=False, indent=2)
    file.write('\n')
print(json.dumps(body_quality, ensure_ascii=False), flush=True)
