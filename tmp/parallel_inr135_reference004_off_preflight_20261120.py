"""独立したOFF試写のsettled窓で2/3実登録と全格子基準を確認する。身体pairは使わない。"""
from pathlib import Path
import csv
import hashlib
import json
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/camera_reference_004_v1'
OUT = BASE / 'independent_off_video_retest02_v1'
REFERENCE = BASE / 'marker_reference_A_current_rotated004.json'
MANIFEST = ROOT / 'tools/logs/pixel_camera/20261120_085933_f0946d18.capture.json'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import iter_video_frames, marker_registration


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as file:
        for chunk in iter(lambda:file.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


OUT.mkdir(exist_ok=False)
manifest, reference = read(MANIFEST), read(REFERENCE)
video = Path(manifest['video_file'])
assert video.name == '20261120_085933_240fps_f880b18f.mp4'
assert manifest['state'] == 'PASSED'
assert digest(video) == manifest['sha256'] and manifest['video_inspection']['sha256_rechecked']
assert set(reference['markers_px']) == {'2','3'}
cv2.setNumThreads(2)
rows = []
for frame_index, frame, stamp in iter_video_frames(video, raw_orientation=True, video_window=(3.3,4.95), frame_stride=4):
    registration = marker_registration(frame, reference)
    ids = set(registration['detected_marker_ids'])
    used = set(registration.get('used_marker_ids', []))
    quality = registration.get('grid_quality', {})
    passed = bool(registration['available'] and ids >= {2,3} and used == {2,3}
        and quality.get('training_points', 0) >= 8 and quality.get('heldout_points', 0) >= 12
        and quality.get('position_rmse_mm', float('inf')) <= .75
        and quality.get('position_p95_mm', float('inf')) <= 1.5
        and quality.get('position_max_mm', float('inf')) <= 2.)
    rows.append(dict(frame=frame_index, time_video_s=stamp, actual_decoded_ids=sorted(ids), used_marker_ids=sorted(used),
        available=registration['available'], registered_grid_passed=passed, reason=registration.get('reason'), grid_quality=quality))
    if len(rows)%30 == 0:
        print(f'Validated {len(rows)}frames / {stamp:.3f}s', flush=True)
assert len(rows) >= 12
passed_count = sum(row['registered_grid_passed'] for row in rows)
decoded_count = sum(set(row['actual_decoded_ids']) >= {2,3} for row in rows)
good = [row['grid_quality'] for row in rows if row['registered_grid_passed']]
result = dict(schema_version=1, purpose='INDEPENDENT_OFF_VIDEO_BOARD_REGISTRATION_PREFLIGHT',
    source_manifest=str(MANIFEST), source_video=str(video), source_reference=str(REFERENCE),
    calibration_source_video='D:/TYPE8_Camera/recordings/remote/20261120_090228_240fps_1e721ca2.mp4',
    independent_video=True, source_capture_state=manifest['state'], window_s=[3.3,4.95], frame_stride=4,
    frames=len(rows), decoded_2_3_frames=decoded_count, decoded_2_3_fraction=decoded_count/len(rows),
    registered_grid_passed_frames=passed_count, registered_grid_passed_fraction=passed_count/len(rows),
    passed=bool(passed_count/len(rows) >= .95 and decoded_count/len(rows) >= .95),
    maximum_observed_grid_rms_mm=max(value['position_rmse_mm'] for value in good) if good else None,
    maximum_observed_grid_p95_mm=max(value['position_p95_mm'] for value in good) if good else None,
    maximum_observed_grid_error_mm=max(value['position_max_mm'] for value in good) if good else None,
    thresholds=dict(registration_fraction=.95, training_points_minimum=8, target_training_points=10,
        heldout_points_minimum=12, grid_rms_mm=.75, grid_p95_mm=1.5, grid_max_mm=2.),
    required_actual_registration_ids=[2,3], ids0_1_used_for_missing_marker_substitution=False,
    body_pair_required=False, body_pose_accepted=False, physical_clearance_verified=False,
    hardware_operations=0, recording_operations=0, raw_config_modified=False,
    input_sha256={str(path):digest(path) for path in (MANIFEST,REFERENCE,video,Path(__file__))}, frames_detail=rows)
with (OUT/'registration_preflight.json').open('x',encoding='utf-8') as file:
    json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False); file.write('\n')
with (OUT/'registration_frames.csv').open('x',encoding='utf-8',newline='') as file:
    flat=[dict(frame=row['frame'], time_video_s=row['time_video_s'], decoded_ids=','.join(map(str,row['actual_decoded_ids'])),
        used_ids=','.join(map(str,row['used_marker_ids'])),passed=row['registered_grid_passed'], reason=row['reason'],
        training_points=row['grid_quality'].get('training_points'),heldout_points=row['grid_quality'].get('heldout_points'),
        rms_mm=row['grid_quality'].get('position_rmse_mm'),p95_mm=row['grid_quality'].get('position_p95_mm'),
        max_mm=row['grid_quality'].get('position_max_mm')) for row in rows]
    writer=csv.DictWriter(file,fieldnames=list(flat[0])); writer.writeheader(); writer.writerows(flat)
print(json.dumps({key:value for key,value in result.items() if key not in ('frames_detail','input_sha256')},ensure_ascii=False,indent=2),flush=True)
