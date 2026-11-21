from pathlib import Path
import hashlib
import json
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1/repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03'
ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003'
RUN=STAGE/'local_execution'/('run_'+ID)
def read(path):return json.loads(path.read_text(encoding='utf-8'))
review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json');trial=read(RUN/'trial.json')
raw=pd.read_csv(RUN/'saved_1ms.csv');stream=pd.read_csv(RUN/'telemetry.csv')
signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=raw.merge(stream[['time_ms']+signature],on=signature)
assert matches.cnt.is_unique and len(matches)>=18
tick,origin=np.polyfit(matches.cnt,matches.time_ms,1)
raw_ms=origin+tick*raw.cnt.to_numpy();gyro=-raw['ego.rad_velo'].to_numpy()
theta=np.concatenate(([0.],np.cumsum(.5*(gyro[1:]+gyro[:-1])*np.diff(raw_ms)/1000)))
lend=raw_ms[-1];end=trial['log_quality']['motion_end_time_ms']
live_ms=stream.time_ms.to_numpy();live_gyro=-stream['ego.rad_velo'].to_numpy()
after_ms=np.r_[lend,live_ms[(live_ms>lend)&(live_ms<end)],end]
after_gyro=np.interp(after_ms,live_ms,live_gyro)
after_theta=np.r_[0.,np.cumsum(.5*(after_gyro[1:]+after_gyro[:-1])*np.diff(after_ms)/1000)]
all_ms=np.r_[raw_ms,after_ms[1:]];all_theta=np.r_[theta,theta[-1]+after_theta[1:]]
zero=review['clock']['saved_zero_video_s']
pose=pd.read_csv(STAGE/'midpoint_brake_clock_v1/midpoint_relative_trajectory.csv')
def mcu(pts):return origin+(pts-zero)*1000
def heading_window(window):
    mask=(pose.time_video_s>=window[0])&(pose.time_video_s<=window[1])&pose.registered_midpoint_valid.astype(bool)
    times=mcu(pose.time_video_s.to_numpy()[mask]);head=np.rad2deg(np.interp(times,all_ms,all_theta))
    return dict(video_window_s=window,samples=len(head),mean_gyro_heading_deg=float(head.mean()),
        min_gyro_heading_deg=float(head.min()),max_gyro_heading_deg=float(head.max()))
entry=heading_window(review['entry_window_pts_s']);exit=heading_window(review['exit_window_pts_s'])
peak=np.abs(raw['ideal.rad_velo'].to_numpy());first=np.flatnonzero(peak>=peak.max()*.02)[0]
body_change=exit['mean_gyro_heading_deg']-entry['mean_gyro_heading_deg']
post={}
for name,stamp in dict(first50ms=lend+50,first100ms=lend+100,
    exit_window_midpoint=mcu(sum(review['exit_window_pts_s'])/2),commanded_motion_end=end).items():
    stamp=min(float(stamp),float(end))
    post[name]=float(np.rad2deg(np.interp(stamp,after_ms,after_theta)))
result=dict(mouse='A',motion='out_r45',source_trial=str(RUN/'trial.json'),
    method='Retained measured-period1ms gyro through Lend, then timestamped5ms telemetry only after Lend; incoming wheel clock from optical review.',
    diagnostic_only=True,no_missing_1ms_samples_reconstructed=True,
    pre_profile_gyro_right_deg=float(np.rad2deg(theta[first])),
    retained_rectangle_integral_profile_to_Lend_deg=read(STAGE/'tracking_1ms_review_v1/verification.json')['gyro_through_lend_measured_time_deg'],
    Lend_mcu_time_ms=float(lend),entry_heading=entry,exit_heading=exit,
    mixed_period_body_heading_change_deg=body_change,optical_exit_path_direction_change_deg=review['exit_path_direction_change_deg'],
    optical_path_minus_mixed_period_body_change_deg=review['exit_path_direction_change_deg']-body_change,
    post_Lend_gyro_right_deg=post,
    limitations=['Path tangent and uncalibrated body yaw are different observations.',
        'Timestamped5ms post-Lend data are diagnostic; no precision1ms gyro claim outside retained samples.',
        'Gyro/body alignment, beta and hardware timing remain uncalibrated. No parameter adoption.'],
    input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (RUN/'saved_1ms.csv',RUN/'telemetry.csv',STAGE/'midpoint_brake_clock_v1/midpoint_review.json')})
with (STAGE/'post_lend_heading_diagnostic.json').open('x',encoding='utf-8') as file:
    json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
