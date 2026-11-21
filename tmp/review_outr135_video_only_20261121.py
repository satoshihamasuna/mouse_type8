"""保存済み不完全ログ試行の光学診断。採用判定・実機操作は行わない。"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
from pixel_camera_geometry import relative_marker_midpoint

run, out = map(lambda p: Path(p).resolve(), sys.argv[1:3])
assert out.is_relative_to((ROOT / 'tools/logs').resolve()) and not out.exists()
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial = read(run / 'trial.json')
settings = read(run / 'telemetry.settings.json')
geometry = read(run / 'body_geometry_used.json')
reference = read(run / 'marker_reference_used.json')
assert trial['state'] == 'INCOMPLETE_LOG'
assert trial['mouse'] == settings['mouse'] == geometry['mouse'] == 'A'
assert trial['motion'] == settings['motion_name'] == 'out_r135'
assert trial['camera_state'] == 'PASSED' and trial['camera_run_start_confirmed']
window = [max(0, trial['video_motion_start_estimate_s'] - 1), trial['recording_limit_seconds']]
out.mkdir()
pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
    marker_reference=reference, video_window=window, marker_detection=geometry.get('marker_detection'),
    rear_colour=geometry['rear_colour'], nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
    marker_span_limits_mm=geometry.get('marker_span_limits_mm'), frame_stride=1,
    progress=lambda stage, count, pts: print(f'{stage}: {count} frames, PTS {pts:.3f}', flush=True))
pose.to_csv(out / 'apparent_floor_marker_pose.csv', index=False)
transform = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), transform)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), transform)
observed = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
relative = relative_marker_midpoint(front, rear, trial['camera_height_mm'],
    geometry['front_height_mm'], geometry['rear_height_mm'], observed)
xy, valid = relative['displacement_xy_mm'], relative['valid']
pd.DataFrame(dict(frame=pose.frame, time_video_s=pose.time_video_s, raw_registered=valid,
    x_mm=xy[:, 0], y_mm=xy[:, 1])).to_csv(out / 'relative_midpoint_diagnostic.csv', index=False)
paths = [run / 'trial.json', run / 'telemetry.settings.json', run / 'marker_reference_used.json',
    run / 'body_geometry_used.json', Path(trial['video_file']), Path(__file__)]
result = dict(state='VIDEO_ONLY_DIAGNOSTIC_NOT_A_COMPLETE_TRIAL', mouse='A', motion='out_r135',
    video_window=window, frame_stride=1, frames=len(pose), raw_registered_observations=int(valid.sum()),
    marker_calibration=pose.attrs['marker_calibration'], body_detection=pose.attrs.get('body_detection', {}),
    parameters=settings['parameters'], full_motion_retained=False, parameter_adoption_allowed=False,
    original_trial_state_preserved=trial['state'], hardware_operations=0,
    input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
(out / 'diagnostic_provenance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: result[k] for k in ('state', 'frames', 'raw_registered_observations') }), flush=True)
