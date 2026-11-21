"""保存動画の未観測フレームの理由だけを確認する。補完・再判定・実機操作なし。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
from collections import Counter
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--stage', type=Path, required=True)
p.add_argument('--trial-id', required=True)
a = p.parse_args()
stage = a.stage.resolve()
assert stage.is_relative_to(ROOT / 'tools/logs/pixel_camera')
run = stage / 'local_execution' / ('run_' + a.trial_id)
out = stage / 'observation_gap_review_v1'
assert not out.exists()
read = lambda x: json.loads(Path(x).read_text(encoding='utf-8-sig'))
trial, review = read(run / 'trial.json'), read(stage / 'entry_clock_midpoint_review_v1/report.json')
pose = pd.read_csv(run / 'full/apparent_floor_marker_pose.csv')
relative = pd.read_csv(run / 'full/relative_marker_midpoint.csv')
assert np.array_equal(pose.frame, relative.frame)
start, end = review['required_window_s']
window = pose.time_video_s.between(start, end)
valid = pose.calibration_valid.fillna(False).astype(bool) & relative.detected.fillna(False).astype(bool)
bad = pose.loc[window & ~valid]
assert abs(float(valid[window].mean()) - review['required_observation_rate']) < 1e-12
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import marker_registration
reference = read(run / 'full/marker_reference_used.json')
capture = cv2.VideoCapture(trial['video_file'])
capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
details, samples = [], []
try:
    for row in bad.itertuples():
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(row.frame))
        ok, frame = capture.read()
        pts = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
        assert ok and abs(pts - row.time_video_s) < .003
        result = marker_registration(frame, reference)
        item = dict(frame=int(row.frame), time_video_s=float(row.time_video_s),
            stored_calibration_valid=bool(row.calibration_valid),
            reason='marker_pair_missing' if row.calibration_valid else result.get('reason', 'unknown'),
            registration_recheck=result)
        details.append(item)
        if len(samples) < 6:
            samples.append((frame, item))
finally:
    capture.release()
out.mkdir()
counts = dict(Counter(item['reason'] for item in details))
result = dict(state='OBSERVATION_GAP_DIAGNOSIS_ONLY', frames=int(window.sum()),
    observed=int(valid[window].sum()), missing=len(details), required_observation_rate=float(valid[window].mean()),
    minimum_required_rate=.95, eligible=False, missing_reasons=counts, details=details,
    observations_imputed=False, original_results_changed=False, thresholds_changed=False,
    physical_commands_sent=0, source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [run / 'trial.json', run / 'full/apparent_floor_marker_pose.csv',
            run / 'full/relative_marker_midpoint.csv', stage / 'entry_clock_midpoint_review_v1/report.json']})
(out / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2,
    default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item()) + '\n', encoding='utf-8')
plt.rcParams['font.family'] = 'Yu Gothic'
fig, axes = plt.subplots(3, 2, figsize=(14, 12), layout='constrained')
for ax in axes.flat:
    ax.axis('off')
for ax, (frame, item) in zip(axes.flat, samples):
    ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    ax.set_title(f"frame {item['frame']} / {item['time_video_s']:.6f}s\n{item['reason']}", fontsize=10)
fig.suptitle('採用しなかった観測の例 / 欠測を補わず、元の判定を保持')
fig.savefig(out / 'rejected_frames.png', dpi=110)
plt.close(fig)
print(json.dumps(dict(output=str(out), frames=result['frames'], observed=result['observed'],
    missing_reasons=counts), ensure_ascii=False))
