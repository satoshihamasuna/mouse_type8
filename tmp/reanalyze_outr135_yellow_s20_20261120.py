"""黄色後点の実色閾値を局所プロファイルで校正し、元動画を基準を保って再解析する。"""
from pathlib import Path
import ast
import json
import subprocess
import sys
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_tuning_report import analysis_video_window, summarize
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
RUN = BASE/'baseline01/local_execution/run_A_OUTR135_1800_baseline_20261120_001'
OUT = BASE/'resume_20261120_v1/yellow_s20_validation_v1'
OUT.mkdir(exist_ok=False)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial = read(RUN/'trial.json')
variant = read(RUN/'body_geometry_used.json')
variant['marker_detection']['yellow']['minimum_saturation'] = 20
variant['marker_detection_revision'] = 'A out135 baseline calibration20261120: keep red unchanged; yellowH25..50/S20/V60/ellipse3/area3, span40..65. Real rear colour components recovered in9 previously missing frames; full-frame verification follows.'
geometry_path = OUT/'body_geometry_A_out135_yellow_s20.json'
with geometry_path.open('x',encoding='utf-8') as file:json.dump(variant,file,ensure_ascii=False,indent=2)
samples = pd.read_csv(BASE/'resume_20261120_v1/missing_frames_review_v1/variant_check_v2/component_pairs.csv')
selected = samples[(samples.variant=='yellow_S20') & samples.originally_missing.fillna(False) & samples.pair.fillna(False)]
assert len(selected) == 9
plt.rcParams['font.family'] = 'Yu Gothic'
fig, axes = plt.subplots(3,3,figsize=(10,8),layout='constrained')
cap = cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
for ax,row in zip(axes.flat,selected.itertuples()):
    cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
    xs=[row.front_x_px,row.rear_x_px];ys=[row.front_y_px,row.rear_y_px]
    x0=max(0,int(min(xs))-12);x1=min(frame.shape[1],int(max(xs))+13)
    y0=max(0,int(min(ys))-12);y1=min(frame.shape[0],int(max(ys))+13)
    ax.imshow(cv2.cvtColor(frame[y0:y1,x0:x1],cv2.COLOR_BGR2RGB),extent=[x0,x1,y1,y0])
    ax.scatter(*xs[:1],s=1) if False else None
    ax.scatter([row.front_x_px],[row.front_y_px],facecolors='none',edgecolors='red',s=65)
    ax.scatter([row.rear_x_px],[row.rear_y_px],facecolors='none',edgecolors='cyan',s=65)
    ax.set_title(f'元フレーム{int(row.frame)}');ax.set_axis_off()
cap.release();fig.suptitle('実画素の色成分：前赤／後シアン輪（推測点なし）')
fig.savefig(OUT/'recovered_colour_components.png',dpi=150);plt.close(fig)
old=samples[(samples.variant=='original')&samples.pair.fillna(False)]
new=samples[(samples.variant=='yellow_S20')&samples.pair.fillna(False)]
same=old.merge(new,on='frame',suffixes=('_old','_new'))
front_delta=np.hypot(same.front_x_px_old-same.front_x_px_new,same.front_y_px_old-same.front_y_px_new)
rear_delta=np.hypot(same.rear_x_px_old-same.rear_x_px_new,same.rear_y_px_old-same.rear_y_px_new)
with (OUT/'sample_validation.json').open('x',encoding='utf-8') as file:
    json.dump(dict(recovered_actual_pairs=9,original_observed_compared=len(same),
        maximum_existing_front_shift_px=float(front_delta.max()),maximum_existing_rear_shift_px=float(rear_delta.max()),
        per_frame_registered_aruco_required=True,minimum_observation_fraction=.95,
        inferred_points_used_as_observations=False,firmware_parameters_changed=False,hardware_operations=0),file,indent=2)

full = OUT/'full'
start,end=analysis_video_window(trial)
command=[sys.executable,str(ROOT/'tools/analysis/pixel_camera_relative_pose.py'),
    '--trial',str(RUN/'trial.json'),'--marker-reference',str(RUN/'marker_reference_used.json'),
    '--body-geometry',str(geometry_path),'--camera-height-mm','1500','--height-source','approximate_for_capture',
    '--fixed-camera','--captured-trial','--video-start',str(start),'--video-end',str(end),
    '--frame-stride','1','--output',str(full)]
with (OUT/'full.stdout.txt').open('x',encoding='utf-8') as log:
    code=subprocess.call(command,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
print(json.dumps(dict(relative_returncode=code,full=str(full))),flush=True)
if code:
    raise SystemExit(code)
try:
    summary=summarize(full)
except Exception as error:
    summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
    with (full/'summary.json').open('x',encoding='utf-8') as file:json.dump(summary,file,ensure_ascii=False,indent=2)

# 既存評価の引数を明示するだけの測定用コピー。品質条件・観測点数は変更しない。
source=ROOT/'tmp/review_outr135_midpoint_trial_20261120.py'
text=source.read_text(encoding='utf-8-sig')
for before,after in {
    "FULL=RUN/'full'":"FULL=Path(sys.argv[3]).resolve()",
    "RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',FULL/'report.json',FULL/'summary.json'":
        "RUN/'marker_reference_used.json',Path(sys.argv[4]),FULL/'report.json',FULL/'summary.json'",
}.items():
    assert text.count(before)==1,before;text=text.replace(before,after)
ast.parse(text)
helper=ROOT/'tmp/review_outr135_midpoint_explicit_profile_20261120.py'
with helper.open('x',encoding='utf-8') as file:file.write(text)
with (OUT/'midpoint.stdout.txt').open('x',encoding='utf-8') as log:
    code=subprocess.call([sys.executable,str(helper),str(RUN),str(OUT/'midpoint_raw_wheel_clock_v1'),str(full),str(geometry_path)],
        stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
print(json.dumps(dict(midpoint_returncode=code,summary=summary,output=str(OUT))),flush=True)
raise SystemExit(code)
