"""赤前マーカー単独の進入/退出方向診断。中点測定・長さの採用には使わない。"""
from pathlib import Path
import sys
import json
import hashlib
import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import colour_components,marker_registration,registration_quad,pose_world_corners,transform_points,board_to_pose_transform
from review_turn1600_A_inr45_20261119 import line_fit

BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_A_INR45_1800_baseline_20261120_002'
OUT=BASE/'parallel_analysis/red_direction_diagnostic_v2'
if OUT.exists(): raise FileExistsError('既存評価を上書きしません')
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial,reference,geometry,report=map(read,(RUN/'trial.json',RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',RUN/'full/report.json'))
saved,stream=pd.read_csv(RUN/'saved_1ms.csv'),pd.read_csv(RUN/'telemetry.csv')
signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=saved.merge(stream[['time_ms']+signature],on=signature)
period,origin=np.polyfit(matches.cnt,matches.time_ms,1)
offset=report['log_time_zero_at_video_s']
motion_start=report['required_motion_window_quality']['video_motion_start_s']
zero=motion_start+(origin-trial['log_quality']['motion_start_time_ms'])/1000
turn_start=zero+period*int(np.flatnonzero(saved['motion.pattern'].to_numpy()==7)[0])/1000
lend=zero+period*saved.cnt.iloc[-1]/1000
stop=report['required_motion_window_quality']['video_motion_end_s']
cv2.setNumThreads(2)
cap=cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
rows=[]
previous=None
index=0
while True:
    ok,frame=cap.read()
    if not ok: break
    stamp=cap.get(cv2.CAP_PROP_POS_MSEC)/1000
    if stamp<motion_start-.2:
        index+=1;continue
    if stamp>stop+.1: break
    registration=marker_registration(frame,reference)
    quad=registration_quad(registration)
    row=dict(frame=index,time_video_s=stamp,calibration_valid=quad is not None,
        decoded_ids=','.join(map(str,registration['detected_marker_ids'])),red_detected=False,
        red_x_mm=np.nan,red_y_mm=np.nan,candidates_on_board=0)
    if quad is not None:
        matrix=np.linalg.inv(board_to_pose_transform(reference))@cv2.getPerspectiveTransform(quad,pose_world_corners(reference.get('board_side_mm',360)))
        components=colour_components(frame,'red',geometry['marker_detection'])
        if components:
            coords=transform_points([v[0] for v in components],matrix)
            eligible=[(p,area) for p,(_,area) in zip(coords,components) if 0<=p[0]<=360 and 0<=p[1]<=360]
            row['candidates_on_board']=len(eligible)
            if eligible:
                if previous is None:
                    point,area=max(eligible,key=lambda v:v[1])
                else:
                    point,area=min(eligible,key=lambda v:np.linalg.norm(v[0]-previous))
                step=0 if previous is None else np.linalg.norm(point-previous)
                if step<=15:
                    row.update(red_detected=True,red_x_mm=float(point[0]),red_y_mm=float(point[1]),component_area_px=int(area))
                    previous=point
    rows.append(row);index+=1
cap.release()
data=pd.DataFrame(rows)
valid=(data.calibration_valid & data.red_detected).to_numpy(bool)
xy=data[['red_x_mm','red_y_mm']].to_numpy()
time=data.time_video_s.to_numpy()
entry_window=[motion_start+.010,turn_start-.008]
exit_window=[lend+.025,stop-.035]
entry_mask=valid & (time>=entry_window[0]) & (time<=entry_window[1])
exit_mask=valid & (time>=exit_window[0]) & (time<=exit_window[1])
entry,eq=line_fit(xy[entry_mask]);exiting,xq=line_fit(xy[exit_mask])
basis=np.column_stack([[-entry[1],entry[0]],entry])
direction=exiting@basis
angle=float(np.degrees(np.arctan2(direction[0],direction[1])))
result=dict(mouse='A',motion='in_r45',status='RED_FRONT_ONLY_DIAGNOSTIC',
    source_trial=str(RUN/'trial.json'),frame_count=len(data),coordinates='board TL origin x toward TR y toward BL; right local is negative board x at downward entry',
    supersedes='red_direction_diagnostic_v1 (legacy pose y-axis was not transformed to board axes; sign error in reported angle)',
    per_frame_registered_red_rate=float(valid.mean()),registration_rate=float(data.calibration_valid.mean()),
    relative_red_path_entry_to_exit_angle_deg=angle,error_from45_deg=angle-45,
    entry_window_pts_s=entry_window,exit_window_pts_s=exit_window,entry_line=eq,exit_line=xq,
    settings_unchanged=True,colour_thresholds_unchanged=True,
    actual_registered_aruco_per_frame=True,missing_registration_policy='exclude_frame',
    absolute_body_yaw_available=False,marker_midpoint_geometry_eligible=False,
    Lstart_Lend_adoption_allowed=False,parameter_adoption_allowed=False,
    limitations=['赤前マーカー単独。黄欠測を補間せず、中点観測率の代用にしない。',
        '静止姿勢のbody yaw校正ではなく、直線上の赤マーカー進行方向の差。',
        '窓は元のモデル時刻を使用。前走距離だけの時刻照合を保証しない。',
        '±15mmの連続性条件と盤内候補を使用。誤認率の独立検証は未実施。'],
    sources_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',Path(__file__)]})
OUT.mkdir()
data.to_csv(OUT/'red_observations.csv',index=False)
(OUT/'diagnostic.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
