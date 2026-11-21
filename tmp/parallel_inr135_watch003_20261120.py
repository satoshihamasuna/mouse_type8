"""IN135の003だけを監視し、保存完了後に1msと汎用front-onsetを評価する。"""
from pathlib import Path
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
LOCAL = BASE / 'baseline01/local_execution'
REQUEST = 'A_INR135_1800_baseline_20261120_003'
RUN = LOCAL / ('run_' + REQUEST)
OUT = BASE / 'parallel_analysis' / REQUEST
print('Waiting only for ' + REQUEST + '; hardware operations 0', flush=True)
while True:
    try:
        trial = json.loads((RUN / 'trial.json').read_text(encoding='utf-8-sig'))
        if trial['state'] == 'CAPTURED_REVIEW_REQUIRED' and trial.get('saved_log', {}).get('status') == 'saved':
            break
    except (OSError, ValueError, KeyError):
        pass
    try:
        job = json.loads((LOCAL / 'jobs' / (REQUEST + '.json')).read_text(encoding='utf-8-sig'))
        if job['state'] in ('DONE', 'FAILED') and not job.get('result', {}).get('execution_write_attempted'):
            print(json.dumps(dict(request=REQUEST, state='NO_SAVED_RUN', result=job.get('result')), ensure_ascii=False), flush=True)
            raise SystemExit(0)
    except (OSError, ValueError, KeyError):
        pass
    time.sleep(2)
OUT.mkdir(parents=True, exist_ok=False)
for script, arguments, label in (
    ('parallel_inr135_tracking_20261120.py', [str(RUN), str(OUT / 'tracking_1ms_v1')], 'tracking'),
    ('parallel_front_run_review_20261120.py', ['--run', str(RUN), '--output', str(OUT / 'front_onset_v1'),
        '--motion', 'in_r135', '--plan', str(BASE / 'baseline01/placement_plan.json'),
        '--midpoint-target', '90', '135', '--front-onset'], 'front_onset')):
    with (OUT / (label + '.stdout.txt')).open('x', encoding='utf-8') as log:
        result = subprocess.run([sys.executable, str(ROOT / 'tmp' / script), *arguments],
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    print(json.dumps(dict(request=REQUEST, analysis=label, returncode=result.returncode, output=str(OUT)), ensure_ascii=False), flush=True)
print('Saved analyses completed; root-managed body-pair/full and hardware unchanged', flush=True)
