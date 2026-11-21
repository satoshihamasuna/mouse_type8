"""元資料を保持して、色検出候補だけを変更した全フレーム再解析を行う。実機操作なし。"""
from pathlib import Path
import argparse
from copy import deepcopy
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from pixel_camera_tuning_report import analysis_video_window

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--run', type=Path, required=True)
p.add_argument('--geometry', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
run, geometry, out = a.run.resolve(), a.geometry.resolve(), a.output.resolve()
assert all(path.is_relative_to(ROOT) for path in (run, geometry, out))
assert not out.exists()
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
trial = read(run / 'trial.json')
original, candidate = read(run / 'body_geometry_used.json'), read(geometry)
back = deepcopy(candidate)
for colour in ('red', 'yellow'):
    back['marker_detection'][colour]['minimum_saturation'] = original['marker_detection'][colour]['minimum_saturation']
assert back == original, '前後マーカーの彩度下限以外を変更しません'
assert candidate['marker_detection']['red']['minimum_saturation'] == 15
assert candidate['marker_detection']['yellow']['minimum_saturation'] in (original['marker_detection']['yellow']['minimum_saturation'], 15)
start, end = analysis_video_window(trial, None, None)
out.mkdir()
command = [sys.executable, str(ROOT / 'tools/analysis/pixel_camera_relative_pose.py'),
    '--trial', str(run / 'trial.json'), '--marker-reference', str(run / 'marker_reference_used.json'),
    '--body-geometry', str(geometry), '--camera-height-mm', str(trial['camera_height_mm']),
    '--height-source', 'approximate_for_capture', '--fixed-camera', '--captured-trial',
    '--video-start', str(start), '--video-end', str(end), '--frame-stride', '1', '--output', str(out / 'full')]
result = subprocess.run(command, check=False)
record = dict(source_trial=str(run / 'trial.json'), candidate_geometry=str(geometry),
    change={colour+'_minimum_saturation':dict(before=original['marker_detection'][colour]['minimum_saturation'],
        after=candidate['marker_detection'][colour]['minimum_saturation']) for colour in ('red', 'yellow')},
    command=command, returncode=result.returncode, hardware_commands_sent=0,
    source_data_changed=False, active_profile_changed=False, parameter_adoption_allowed=False,
    quality_thresholds_changed=False, inferred_observations_allowed=False,
    source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (run / 'trial.json', run / 'body_geometry_used.json', geometry, run / 'marker_reference_used.json')})
with (out / 'candidate_analysis_provenance.json').open('x', encoding='utf-8') as f:
    json.dump(record, f, ensure_ascii=False, indent=2)
print(json.dumps(dict(output=str(out), analysis_returncode=result.returncode), ensure_ascii=False))
