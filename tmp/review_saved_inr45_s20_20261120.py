"""保存動画を低彩度黄マーカーの候補条件で再解析する。実機接続なし。"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--geometry', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
run, geometry, out = args.run.resolve(), args.geometry.resolve(), args.output.resolve()
assert all(p.is_relative_to(BASE) for p in (run, geometry, out))
assert not out.exists()
trial_path = run / 'trial.json'
trial = json.loads(trial_path.read_text(encoding='utf-8'))
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED'
assert trial['mouse'] == 'A' and trial['motion'] == 'in_r45' and trial['preset'] == 1800
command = [sys.executable, str(ROOT / 'tools/analysis/pixel_camera_relative_pose.py'),
    '--trial', str(trial_path), '--marker-reference', str(run / 'marker_reference_used.json'),
    '--body-geometry', str(geometry), '--camera-height-mm', '1500',
    '--height-source', 'approximate_for_capture', '--fixed-camera', '--captured-trial',
    '--video-start', '8.8', '--video-end', '10.4', '--frame-stride', '1', '--output', str(out)]
out.parent.mkdir(parents=True, exist_ok=True)
with out.with_suffix('.stdout.txt').open('x', encoding='utf-8') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
        env=dict(os.environ, PYTHONUTF8='1', OPENCV_FOR_THREADS_NUM='4'),
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
assert result.returncode in (0, 2), result.returncode
from pixel_camera_tuning_report import summarize
try:
    summary = summarize(out)
except ValueError as error:
    summary = dict(status='NEEDS_OPERATOR_REVIEW', auto_stop_confirmed=False, error=str(error))
    with (out / 'summary.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
report = json.loads((out / 'report.json').read_text(encoding='utf-8'))
print(json.dumps(dict(output=str(out), returncode=result.returncode, summary=summary,
    required_motion_quality=report.get('required_motion_window_quality'),
    robot_connections=0), ensure_ascii=False), flush=True)
