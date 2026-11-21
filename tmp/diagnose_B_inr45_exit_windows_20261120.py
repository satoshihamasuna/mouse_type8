"""保存済み軌道で出口角の測定窓感度を診断する。既存の採否条件は変更しない。"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('runs',nargs='+')
args=parser.parse_args()
OUT=BASE/'exit_window_diagnostic_v1';OUT.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
results=[]
for sub in args.runs:
    run=(BASE/sub).resolve();assert run.is_relative_to(BASE)
    report=read(run/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
    assert report['independent_optical_geometry_eligible']
    raw,stream=pd.read_csv(run/'saved_1ms.csv'),pd.read_csv(run/'telemetry.csv')
    pose=pd.read_csv(run/'midpoint_raw_wheel_clock_v3/midpoint_relative_trajectory.csv')
    signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
    matched=raw.merge(stream[['time_ms']+signature],on=signature)
    tick,origin=np.polyfit(matched.cnt,matched.time_ms,1)
    mcu_lend=origin+tick*raw.cnt.iloc[-1]
    lend=report['Lend_video_pts_s']
    after=stream[stream.time_ms>=mcu_lend]
    decel=after[(after['motion.pattern']==2)&(after['ideal.accel']<-.1)]
    assert len(decel)
    decel_video=lend+(decel.time_ms.iloc[0]-mcu_lend)/1000
    t=pose.time_video_s.to_numpy();xy=pose[['right_mm','forward_mm']].to_numpy()
    valid=pose.registered_midpoint_valid.to_numpy(bool)
    windows=[('published',*report['exit_window_pts_s']),
        ('early_post_0_40ms',lend,lend+.040),('post_15_35ms',lend+.015,lend+.035),
        ('after_12ms_before_decel',lend+.012,decel_video-.003)]
    rows=[]
    for name,start,end in windows:
        points=xy[valid&(t>=start)&(t<=end)]
        row=dict(name=name,start_ms_after_Lend=(start-lend)*1000,end_ms_after_Lend=(end-lend)*1000,count=len(points))
        if len(points)>=3:
            direction=np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]
            if direction@(points[-1]-points[0])<0:direction=-direction
            residual=points-points.mean(0);residual-=np.outer(residual@direction,direction)
            row.update(angle_deg=float(np.degrees(np.arctan2(direction[0],direction[1]))),
                span_mm=float(np.ptp(points@direction)),rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))))
        rows.append(row)
    result=dict(source_trial=str(run/'trial.json'),published_exit_angle_deg=report['exit_path_direction_change_deg'],
        windows=rows,acceptance_changed=False,parameter_adoption_allowed=False,
        note='短い窓は観測数と移動範囲を示す診断のみ。既存の全フレーム品質・直線採否条件の代用にしない。',
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            run/'midpoint_raw_wheel_clock_v3/midpoint_review.json',run/'midpoint_raw_wheel_clock_v3/midpoint_relative_trajectory.csv',run/'saved_1ms.csv',run/'telemetry.csv')})
    destination=OUT/(run.parents[1].name+'.json')
    with destination.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    results.append(result)
print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)
