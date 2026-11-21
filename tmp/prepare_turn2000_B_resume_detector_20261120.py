"""B2000再開用の両色観測モデルを保存動画で検証する。機器への通信なし。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import sys

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_static_pose import measure
from video_marker_pose import read_video_pose

BASE = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
RUN = BASE/'cli_green_session01/run_B_R90_2000_omKi005_green_20261120_001'
OTHER = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT = BASE/'resume_20261120_detector_v1'
OUT.mkdir(exist_ok=False)
read = lambda p: json.loads(Path(p).read_text(encoding='utf-8'))
write = lambda p, d: Path(p).write_text(json.dumps(d, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

model = OUT/'body_model_B_green_red.npz'
shutil.copyfile(ROOT/'tools/logs/pixel_camera/models/body_model_B_green_red.npz', model)
geometry = read(RUN/'green_tracking_probe_v4/body_geometry_candidate.json')
geometry['marker_detection']['body_model'] = dict(enabled=True, model_file=str(model),
    model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(), require_colour_pair=True)
geometry['marker_detection_source'] = 'Resume candidate: v4 colour separation with existing B green/red body model; both colour candidates required in each image. No single-marker inferred points. Not adopted until saved and current footage checks.'
geometry_path = OUT/'body_geometry_candidate.json'
write(geometry_path, geometry)
reference = read(RUN/'marker_reference_used.json')
reference_path = OUT/'marker_reference.json'
write(reference_path, reference)
config = read(BASE/'green_handoff_preparation_v1/config.json')
config.update(reference=str(reference_path), geometry=str(geometry_path))
write(OUT/'config.candidate.json', config)
settings = read(BASE/'candidate_omKi0p005.settings.json')
settings['source'] = 'Resume B2000 long_r90 Ki0.005; same turn/PID/FF candidate as green run001. RAM must be restored and verified after power-on. No execution in preparation.'
write(OUT/'candidate.settings.json', settings)

results = []
for tag in ('B_green_front_OFF01', 'B_green_front_OFF02'):
    capture = read(OTHER/(tag+'_static_capture.json'))
    original = read(OTHER/(tag+'_static_check.json'))
    result = measure(Path(capture['video_file']), reference, geometry, original['placement'])
    result.update(video_file=capture['video_file'], source_check=str(OTHER/(tag+'_static_check.json')))
    write(OUT/(tag+'_check.json'), result)
    results.append(dict(kind='independent_static', name=tag, passed=result['passed'],
        observed=result['observed'], frames=result['frames'], rms_deg=result.get('heading_jitter_rms_deg')))
    print(json.dumps(results[-1]), flush=True)

for name, folder, analysis in [('source2000', RUN, 'full_colour_candidate_v4'),
        ('independent1600', OTHER/'cli_session04/run_B_R90_1600_Lstart13p6_green_20261120_001', 'full')]:
    trial = read(folder/'trial.json')
    report = read(folder/analysis/'report.json')
    window = report['required_motion_window_quality']['required_interval_pts_s']
    marker_reference = read(folder/'marker_reference_used.json')
    pose, _, _, _ = read_video_pose(Path(trial['video_file']), 'green',
        marker_reference=marker_reference, video_window=window, frame_stride=1,
        marker_detection=geometry['marker_detection'], rear_colour='red',
        nominal_marker_span_mm=50, marker_span_limits_mm=[40, 65],
        progress=lambda stage, count, pts: print(name, stage, count, pts, flush=True))
    pose.to_csv(OUT/(name+'_motion_pose.csv'), index=False)
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    result = dict(kind='motion', name=name, frames=len(pose), observed=int(valid.sum()),
        observation_rate=float(valid.mean()), calibration_rate=float(pose.calibration_valid.mean()),
        passed=bool(valid.mean() >= .95 and pose.calibration_valid.mean() >= .95),
        video_file=trial['video_file'], video_window=window, manual_marker_identity='pending')
    write(OUT/(name+'_motion_check.json'), result)
    results.append(result)
    print(json.dumps(result), flush=True)

write(OUT/'saved_video_validation.json', dict(state='SAVED_VIDEO_CHECKS_COMPLETE',
    results=results, quality_checks_passed=all(r['passed'] for r in results),
    manual_marker_identity_required=True, current_static_check_required=True,
    candidate_adopted=False, device_commands_sent=0,
    source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/'tools/body_appearance_model.py', ROOT/'tools/analysis/video_marker_pose.py', model, geometry_path)}))
