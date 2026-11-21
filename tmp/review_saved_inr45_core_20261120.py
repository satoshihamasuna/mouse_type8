"""保存済みA1800 in_r45の動作全区間を含む明示窓を既存CLIで解析する。"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trial', type=Path, required=True)
parser.add_argument('--video-start', type=float, required=True)
parser.add_argument('--video-end', type=float, required=True)
parser.add_argument('--name', required=True)
args = parser.parse_args()
trial_path = args.trial.resolve()
assert trial_path.is_relative_to(ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1')
assert args.name.isidentifier()
trial = json.loads(trial_path.read_text(encoding='utf-8'))
assert trial['mouse'] == 'A' and trial['motion'] == 'in_r45' and trial['preset'] == 1800
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED'
output = trial_path.parent / args.name
assert not output.exists()
command = [sys.executable, str(ROOT / 'tools/analysis/pixel_camera_tuning_report.py'),
    '--trial', str(trial_path), '--stride', '1',
    '--video-start', str(args.video_start), '--video-end', str(args.video_end),
    '--output', str(output)]
with (trial_path.parent / (args.name + '.stdout.txt')).open('x', encoding='utf-8') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
        env=dict(os.environ, PYTHONUTF8='1', OPENCV_FOR_THREADS_NUM='4'),
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
print(json.dumps(dict(returncode=result.returncode, output=str(output),
    video_window=[args.video_start, args.video_end], frame_stride=1,
    robot_connections=0), ensure_ascii=False), flush=True)
raise SystemExit(result.returncode)
