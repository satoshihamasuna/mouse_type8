from pathlib import Path
from datetime import datetime, timezone
from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE = BASE / 'fixed_lengths_cplus0p1_Ls19p5_Le18p8_20261121_01'
ID = 'A_OUTR45_1800_cplus0p1_Ls19p5_Le18p8_20261121_001'
RUN = STAGE / 'local_execution' / ('run_' + ID)
PYTHON = 'D:/venv/venv3-11/Scripts/python.exe'

def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2)
        file.write('\n')

confirmation = dict(request_id=ID, both_powered_off=True, contact_detected=False,
    source='operator_report', confirmed_at=datetime.now(timezone.utc).isoformat(),
    operator_answer='停止・接触なし・両方OFFで待機')
save(STAGE / 'local_execution/operator_results' / (ID + '.json'), confirmation)
save(RUN / 'operator_postrun_confirmation.json', confirmation)

tracking = ROOT / 'tmp/review_outr45_cplus0p1_tracking_20261121.py'
source = (ROOT / 'tmp/review_outr45_joint01_tracking_20261121.py').read_text(encoding='utf-8')
old = 'joint_cminus2p0_Ls19p5_Le18p8_20261121_01/tracking_1ms_review_v1'
assert source.count(old) == 1
with tracking.open('x', encoding='utf-8') as file:
    file.write(source.replace(old, STAGE.name + '/tracking_1ms_review_v1'))

full_command = [PYTHON, 'tools/analysis/pixel_camera_tuning_report.py', '--trial', str(RUN/'trial.json'),
    '--stride', '1', '--video-start', '7.5', '--video-end', '11.5']
track_command = [PYTHON, str(tracking), str(RUN.relative_to(ROOT))]
save(STAGE / 'full_analysis_plan.json', dict(full_command=full_command, tracking_command=track_command,
    video_window_reason='Motion estimate8.0827s, command end8.4647s, telemetry stationary begins10.2807s; includes pre-motion and after-brake stationary windows.',
    frame_stride=1, physical_quality_thresholds_unchanged=True, no_hardware_operations=True,
    operator_confirmation=confirmation))
env = os.environ.copy()
env.update(PYTHONUTF8='1', OPENCV_FOR_THREADS_NUM='4')

def execute(label, command, output):
    with output.open('x', encoding='utf-8') as file:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=file, stderr=subprocess.STDOUT)
    print(label, result.returncode, str(output), flush=True)
    return result.returncode

with ThreadPoolExecutor(max_workers=2) as pool:
    full = pool.submit(execute, 'full', full_command, RUN/'full.stdout.txt')
    track = pool.submit(execute, 'tracking', track_command, STAGE/'tracking_1ms.stdout.txt')
    codes = dict(full=full.result(), tracking=track.result())
save(STAGE/'analysis_execution_result.json', codes)
print(json.dumps(codes), flush=True)
