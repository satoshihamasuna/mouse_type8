from pathlib import Path
import hashlib
import json
import sys
import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from video_marker_pose import iter_video_frames
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
OUT=BASE/'firmware_outR45_adoption_20261121_v1/selected_visual_review_v1'
OUT.mkdir(exist_ok=False)
def read(path):return json.loads(path.read_text(encoding='utf-8'))
sources=[]
for number,(folder_name,identifier) in enumerate((
    ('lengths_cminus1p7_Ls15p4_Le15p9_20261121_01','A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_001'),
    ('repeat_cminus1p7_Ls15p4_Le15p9_20261121_02','A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002')),1):
    stage=BASE/folder_name;run=stage/'local_execution'/('run_'+identifier)
    trial=read(run/'trial.json');review=read(stage/'midpoint_brake_clock_v1/midpoint_review.json')
    video=Path(trial['video_file']);pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    window=review['independent_clock_required_motion_quality'];start=window['video_motion_start_s'];end=window['video_motion_end_s']
    raw=pd.read_csv(run/'saved_1ms.csv');turn=np.flatnonzero(abs(raw['ideal.rad_velo'].to_numpy())>.02*abs(raw['ideal.rad_velo']).max())
    turn_start=review['clock']['saved_zero_video_s']+turn[0]*review['clock']['measured_tick_ms']/1000
    turn_end=review['clock']['saved_zero_video_s']+turn[-1]*review['clock']['measured_tick_ms']/1000
    labels=['pre-motion','entry','turn-start','turn-middle','Lend','exit','motion-end','stopped']
    wanted=[start-.08,sum(review['entry_window_pts_s'])/2,turn_start,(turn_start+turn_end)/2,
        review['Lend_video_pts_s'],sum(review['exit_window_pts_s'])/2,end+.02,sum(review['late_stationary']['window_pts_s'])/2]
    selected={}
    for label,stamp in zip(labels,wanted):
        valid=pose.marker_detected_raw.astype(bool)&pose.calibration_valid.astype(bool)
        row=pose.loc[valid].iloc[np.argmin(abs(pose.time_video_s[valid].to_numpy()-stamp))]
        selected[int(row.frame)]=(label,row)
    by_frame=pose.set_index('frame');crop_images={};full_trace_image=None
    trace=[];all_trace=[];selected_records=[]
    writer=cv2.VideoWriter(str(OUT/f'run{number}_observed_trace_slow8x.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720))
    assert writer.isOpened()
    for index,frame,pts in iter_video_frames(video,True,(start-.12,review['late_stationary']['window_pts_s'][1]),1):
        cached=by_frame.loc[index] if index in by_frame.index else None
        observed=cached is not None and bool(cached.marker_detected_raw) and bool(cached.calibration_valid)
        if cached is not None:assert abs(float(cached.time_video_s)-pts)<1e-6
        if start-.01<=pts<=end+.02:
            if observed:
                point=tuple(np.rint([cached.midpoint_image_x_px,cached.midpoint_image_y_px]).astype(int))
                if trace and index!=trace[-1][0]+1:all_trace.append(trace);trace=[]
                trace.append((index,point))
            elif trace:all_trace.append(trace);trace=[]
        overlay=frame.copy()
        for segment in all_trace+[trace]:
            if len(segment)>1:cv2.polylines(overlay,[np.array([value[1] for value in segment],np.int32)],False,(255,220,0),2,cv2.LINE_AA)
        cv2.putText(overlay,f'Run{number} actual midpoint trace / PTS{pts:.4f}s / playback8x slower',(18,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
        writer.write(overlay)
        if index in selected:
            label,row=selected[index];cv2.imwrite(str(OUT/f'run{number}_{label}_raw.png'),frame)
            x,y=np.rint([row.midpoint_image_x_px,row.midpoint_image_y_px]).astype(int)
            x0=max(0,min(x-110,1280-220));y0=max(0,min(y-90,720-180))
            crop=cv2.resize(frame[y0:y0+180,x0:x0+220],(440,360),interpolation=cv2.INTER_NEAREST)
            panel=np.zeros((398,440,3),np.uint8);panel[38:]=crop
            cv2.putText(panel,f'{label} / {pts:.4f}s',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
            crop_images[label]=panel
            selected_records.append(dict(label=label,source_frame=index,source_pts_s=pts,raw_png=f'run{number}_{label}_raw.png',crop_origin_px=[int(x0),int(y0)]))
            if label=='motion-end':full_trace_image=overlay.copy()
    writer.release();assert set(crop_images)==set(labels)
    sheet=np.vstack([np.hstack([crop_images[label] for label in labels[i:i+4]]) for i in (0,4)])
    cv2.imwrite(str(OUT/f'run{number}_contact_sheet.png'),sheet)
    assert full_trace_image is not None;cv2.imwrite(str(OUT/f'run{number}_observed_trace.png'),full_trace_image)
    sources.append(dict(run=number,video=str(video),video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
        selected_frames=selected_records,measured_optical_angle_deg=review['exit_path_direction_change_deg'],
        observed_pixels_only=True,planned_or_inferred_path_drawn=False,frame_stride=1,
        overlay_video_playback_slowdown=8,raw_pngs_unmodified=True))
with (OUT/'manifest.json').open('x',encoding='utf-8') as file:
    json.dump(dict(purpose='Raw-frame visual trajectory review, not a replacement for metric registration.',sources=sources),file,ensure_ascii=False,indent=2)
print(str(OUT),flush=True)
