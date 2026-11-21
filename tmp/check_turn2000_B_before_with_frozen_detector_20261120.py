"""補正前の動作窓を補正後と同じ保存済み検出コードで照合する。実機通信なし。"""
from pathlib import Path
import importlib.util
import json
import sys

import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
BASE=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
AFTER=BASE/'next_correction_plus1p5_20261120_v1/run_session01/run_B_R90_2000_correction1p5_20261120_001'
BEFORE=BASE/'resume_entry_relative_20261120_v1/run_session01/run_B_R90_2000_omKi005_green_relative_20261120_003'
SNAP=AFTER/'analysis_source_used'
OUT=AFTER/'before_frozen_detector_review'
OUT.mkdir(exist_ok=False)
for name in ('pixel_camera_geometry','pixel_camera_reference','marker_detection','body_appearance_model','video_marker_pose'):
    spec=importlib.util.spec_from_file_location(name,SNAP/(name+'.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
import video_marker_pose as detector
from pixel_camera_geometry import relative_marker_midpoint
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial=read(BEFORE/'trial.json');geometry=read(BEFORE/'body_geometry_used.json');reference=read(BEFORE/'marker_reference_used.json')
quality=read(BEFORE/'full/report.json');previous=read(BEFORE/'geometry_review_v1/video_comparison.json')['comparisons'][0]
window=quality['required_motion_window_quality']['required_interval_pts_s']
pose,_,_,_=detector.read_video_pose(Path(trial['video_file']),'green',marker_reference=reference,
    video_window=window,frame_stride=1,marker_detection=geometry['marker_detection'],rear_colour='red',
    nominal_marker_span_mm=50,marker_span_limits_mm=[40,65],
    progress=lambda stage,count,pts:print(stage,count,pts,flush=True))
pose.to_csv(OUT/'pose.csv',index=False)
old=pd.read_csv(BEFORE/'full/apparent_floor_marker_pose.csv')
columns=['blue_x_mm','blue_y_mm','yellow_x_mm','yellow_y_mm']
joined=pose[['frame',*columns]].merge(old[['frame',*columns]],on='frame',suffixes=('_frozen','_previous'))
difference=np.column_stack([joined[c+'_frozen']-joined[c+'_previous'] for c in columns])
inv=np.linalg.inv(detector.board_to_pose_transform(reference))
front=detector.transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),inv)
rear=detector.transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),inv)
valid=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)
xy=relative_marker_midpoint(front,rear,1500,10,1,valid)['displacement_xy_mm']
stamps=pose.time_video_s.to_numpy()


def line(points):
    direction=np.linalg.svd(points-points.mean(axis=0),full_matrices=False)[2][0]
    return direction if direction@(points[-1]-points[0])>0 else -direction


results=[]
for delta in (-.005,0.,.005):
    directions=[line(xy[valid&(stamps>=w[0]+delta)&(stamps<=w[1]+delta)])
        for w in (previous['entry_window_pts_s'],previous['exit_window_pts_s'])]
    first,last=directions
    angle=float(np.degrees((np.arctan2(last[1],last[0])-np.arctan2(first[1],first[0])+np.pi)%(2*np.pi)-np.pi))
    results.append(dict(window_shift_s=delta,path_rotation_deg=angle))
result=dict(source_trial=str(BEFORE/'trial.json'),frozen_source_manifest=str(SNAP/'manifest.json'),
    frames=len(pose),observed=int(valid.sum()),observation_rate=float(valid.mean()),
    maximum_coordinate_difference_from_previous_mm=float(np.nanmax(np.abs(difference))),
    direction_results=results,previous_angle_deg=previous['entry_to_exit_path_rotation_deg'],
    scope='Same straight windows and measured-entry direction; only detector implementation changed. No scale fit or independent hardware sync.')
(OUT/'report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result),flush=True)
