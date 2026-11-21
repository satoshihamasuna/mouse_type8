"""保存済み2試行を同一の黄色検出候補で再評価。録画・実機指令なし。"""
from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
label = sys.argv[1]
runs = dict(baseline='cli_session03/run_A_R90_1600_baseline_20261118_001',
            corr2p5='cli_session04/run_A_R90_1600_corr2p5_20261118_001')
RUN = BASE / runs[label]
OUT = BASE / 'analysis_yellow_v2' / label
profile = BASE / 'analysis_yellow_v2/body_geometry_A_yellow_v2.json'
if OUT.exists():
    raise RuntimeError('既存解析は上書きしません')
provenance = json.loads((RUN / 'full/apparent_floor_marker_pose.provenance.json').read_text(encoding='utf-8'))
start, end = provenance['video_window']
command = [sys.executable, str(ROOT / 'tools/analysis/pixel_camera_relative_pose.py'),
    '--trial', str(RUN / 'trial.json'), '--marker-reference', str(RUN / 'marker_reference_used.json'),
    '--body-geometry', str(profile), '--camera-height-mm', '1500',
    '--height-source', 'approximate_for_capture', '--fixed-camera', '--captured-trial',
    '--video-start', str(start), '--video-end', str(end), '--frame-stride', '1', '--output', str(OUT)]
attempt = sys.argv[2] if len(sys.argv) > 2 else 'attempt01'
logfile = OUT.parent / (label + '.' + attempt + '.stdout.txt')
print('Analyzing saved video:', label, flush=True)
with logfile.open('x', encoding='utf-8') as output:
    result = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, cwd=ROOT)
if result.returncode:
    print('Analysis failed:', result.returncode, logfile, flush=True)
    raise SystemExit(result.returncode)
sys.path[:0] = [str(ROOT / 'tools/analysis'), str(ROOT / 'tools')]
from pixel_camera_tuning_report import summarize
summary = summarize(OUT)
print(json.dumps(summary, ensure_ascii=False), flush=True)
