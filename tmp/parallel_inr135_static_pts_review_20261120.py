"""失敗した静止動画の実PTSを窓別に読み、元manifestを保持して局所姿勢を測る。"""
from pathlib import Path
import csv
import hashlib
import json
import subprocess
import av

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'parallel_analysis/static_request001_review_v1'
MANIFEST = ROOT / 'tools/logs/pixel_camera/20261120_075638_ba4e4634.capture.json'
PREPARE = BASE / 'baseline01/local_execution/prepare_A_INR135_1800_baseline_20261120_001'
OUT.mkdir(parents=True, exist_ok=False)
read = lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
manifest_bytes = MANIFEST.read_bytes()
manifest = read(MANIFEST)
video = Path(manifest['video_file'])
assert manifest['state'] == 'FAILED'
digest = hashlib.sha256(video.read_bytes()).hexdigest()
assert digest == manifest['sha256'] == manifest['video_inspection']['sha256']
assert manifest['video_inspection']['sha256_rechecked'] is True
rows = []
with av.open(str(video)) as container:
    for index, frame in enumerate(container.decode(video=0)):
        rows.append(dict(frame=index, pts_seconds=float(frame.pts*frame.time_base) if frame.pts is not None else None,
            corrupt=bool(frame.is_corrupt), width=frame.width, height=frame.height))
assert all(row['pts_seconds'] is not None for row in rows)
gaps = []
for previous, current in zip(rows, rows[1:]):
    interval = current['pts_seconds']-previous['pts_seconds']
    if interval > 1.5/240 or interval <= 0:
        gaps.append(dict(before_frame=previous['frame'], after_frame=current['frame'],
            before_s=previous['pts_seconds'], after_s=current['pts_seconds'], interval_ms=interval*1000))
selected = [row for row in rows if 3.3 <= row['pts_seconds'] <= 4.95]
crossing = [gap for gap in gaps if gap['before_s'] <= 4.95 and gap['after_s'] >= 3.3]
report = dict(source_manifest=str(MANIFEST), source_manifest_state_unchanged='FAILED',
    video=str(video), video_sha256=digest, transfer_sha_verified=True, total_frames=len(rows),
    first_pts_s=rows[0]['pts_seconds'], last_pts_s=rows[-1]['pts_seconds'], gaps=gaps,
    static_window_s=[3.3,4.95], static_window_frames=len(selected),
    static_window_first_pts_s=selected[0]['pts_seconds'], static_window_last_pts_s=selected[-1]['pts_seconds'],
    gaps_inside_or_crossing_static_window=crossing,
    static_window_max_internal_interval_ms=max(b['pts_seconds']-a['pts_seconds'] for a,b in zip(selected,selected[1:]))*1000,
    full_window_covered=rows[0]['pts_seconds'] <= 3.3 and rows[-1]['pts_seconds'] >= 4.95,
    corrupt_frames=sum(row['corrupt'] for row in rows), wrong_size_frames=sum((row['width'],row['height']) != (1280,720) for row in rows),
    execution_write_attempted=False, original_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest())
with (OUT/'pts_review.json').open('x',encoding='utf-8') as file:
    json.dump(report,file,ensure_ascii=False,indent=2);file.write('\n')
with (OUT/'decoded_pts.csv').open('x',encoding='utf-8',newline='') as file:
    writer=csv.DictWriter(file,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print(json.dumps(report,ensure_ascii=False),flush=True)
plan=dict(video=str(video), camera_manifest=str(MANIFEST),
    placement=read(BASE/'baseline01/placement_plan.json')['placement'], reference=str(PREPARE/'reference.used.json'),
    geometry=str(PREPARE/'geometry.used.json'), stationary_video_window_s=[3.3,4.95], stationary_frame_stride=4)
with (OUT/'static_plan.json').open('x',encoding='utf-8') as file:
    json.dump(plan,file,ensure_ascii=False,indent=2);file.write('\n')
with (OUT/'static.stdout.txt').open('x',encoding='utf-8') as log:
    result=subprocess.run(['D:/venv/venv3-11/Scripts/python.exe',str(BASE/'baseline01/static_pose_after_settle.py'),
        'measure',str(OUT/'static_plan.json'),str(OUT/'static_check.json')],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NO_WINDOW)
assert MANIFEST.read_bytes() == manifest_bytes
print(json.dumps(dict(static_measure_returncode=result.returncode,static_result=str(OUT/'static_check.json'),
    original_manifest_unchanged=True)),flush=True)
