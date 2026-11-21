"""保存済みの時刻対応で出口窓の点数・距離・方向感度を記録する。実機操作なし。"""
from pathlib import Path
import json
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_B_R180_1600_baseline_green_20261120_001'
OUT=BASE/'baseline001_exit_window_diagnostic.json'
assert not OUT.exists()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
trial=read(RUN/'trial.json')
saved=pd.read_csv(RUN/'saved_1ms.csv');stream=pd.read_csv(RUN/'telemetry.csv')
sig=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=saved.merge(stream[['time_ms']+sig],on=sig,how='inner')
period,origin=np.polyfit(matches.cnt,matches.time_ms,1)
alignment=pd.read_csv(BASE/'baseline001_video_review_v1/baseline001_encoder_alignment.csv')
offset=float(np.median(alignment.time_video_s-alignment.time_log_s))
live_start=trial['log_quality']['motion_start_time_ms']-500
first_mcu=float(stream[stream.time_ms>=live_start].time_ms.iloc[0])
pts=lambda m:offset+(m-first_mcu)/1000
lend=pts(origin+period*(len(saved)-1));stop=pts(trial['log_quality']['motion_end_time_ms'])
rel=pd.read_csv(RUN/'full/relative_marker_midpoint.csv')
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
t=pose.time_video_s.to_numpy()
valid=rel.detected.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)
xy=rel[['center_dx_mm','center_dy_mm']].to_numpy()
rows=[]
for name,start,end in [('original_8ms_margin',lend+.008,stop-.008),
    ('full_controlled_post_run',lend,stop),('exclude_first4ms',lend+.004,stop),
    ('physical_tail_plus20ms',lend+.004,stop+.020),('physical_tail_plus40ms',lend+.008,stop+.040)]:
    p=xy[valid&(t>=start)&(t<=end)]
    d=np.linalg.svd(p-p.mean(axis=0),full_matrices=False)[2][0]
    if d@(p[-1]-p[0])<0:d=-d
    residual=p-p.mean(axis=0);residual-=np.outer(residual@d,d)
    rows.append(dict(name=name,window_s=[start,end],count=len(p),
        span_mm=float(np.ptp(p@d)),heading_board_deg=float(np.degrees(np.arctan2(d[1],d[0]))%360),
        line_rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
        meets_existing_15point_30mm_requirement=bool(len(p)>=15 and np.ptp(p@d)>=30)))
with OUT.open('x',encoding='utf-8') as output:
    json.dump(dict(period_ms=float(period),lend_pts_s=lend,post_ideal_stop_pts_s=stop,
        source_alignment=str(BASE/'baseline001_video_review_v1/baseline001_encoder_alignment.csv'),windows=rows),
        output,ensure_ascii=False,indent=2)
print(json.dumps(rows,ensure_ascii=False),flush=True)
