from pathlib import Path
import hashlib
import json
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from video_marker_pose import board_to_pose_transform,transform_points
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
OUT=BASE/'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03/visual_trajectory_review_v1'
rows=[];fig,ax=plt.subplots(figsize=(7,7),layout='constrained')
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def line(points):
    direction=np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]
    if direction@(points[-1]-points[0])<0:direction=-direction
    return float(np.degrees(np.arctan2(direction[1],direction[0]))%360)
for number,(name,identifier) in enumerate((
    ('retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02','A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002'),
    ('repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03','A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003')),1):
    stage=BASE/name;run=stage/'local_execution'/('run_'+identifier);full=run/'full'
    review=read(stage/'midpoint_brake_clock_v1/midpoint_review.json')
    pose=pd.read_csv(full/'apparent_floor_marker_pose.csv')
    inverse=np.linalg.inv(board_to_pose_transform(read(full/'marker_reference_used.json')))
    front=transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),inverse)
    rear=transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),inverse)
    midpoint=(front+rear)/2;time=pose.time_video_s.to_numpy()
    valid=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)&np.isfinite(midpoint).all(1)
    def points(window):return midpoint[valid&(time>=window[0])&(time<=window[1])]
    start=review['independent_clock_required_motion_quality']['video_motion_start_s']
    end=review['independent_clock_required_motion_quality']['video_motion_end_s']
    initial=np.median(points([start-.18,start-.04]),axis=0)
    lend=np.array([np.interp(review['Lend_video_pts_s'],time[valid],axis[valid]) for axis in midpoint.T])
    entry_heading=line(points(review['entry_window_pts_s']));exit_heading=line(points(review['exit_window_pts_s']))
    active=valid&(time>=start-.03)&(time<=end+.03)
    ax.plot(*midpoint[active].T,label=f'Run{number} apparent midpoint')
    ax.scatter(*initial,marker='o',s=30);ax.scatter(*lend,marker='s',s=40)
    row=dict(run=number,initial_apparent_board_midpoint_mm=initial.tolist(),
        initial_position_residual_from315_90_mm=(initial-[315,90]).tolist(),
        board_entry_path_heading_deg=entry_heading,planned_entry_heading_deg=135.,
        entry_direction_error_deg=entry_heading-135.,board_exit_path_heading_deg=exit_heading,
        planned_exit_heading_deg=180.,exit_direction_error_deg=exit_heading-180.,
        apparent_board_Lend_mm=lend.tolist(),planned_board_Lend_mm=[135.,225.],
        apparent_board_Lend_residual_mm=(lend-[135,225]).tolist(),
        metric_relative_turn_angle_deg=review['exit_path_direction_change_deg'],
        no_origin_rotation_scale_or_endpoint_fit=True,source_review=str(stage/'midpoint_brake_clock_v1/midpoint_review.json'),
        source_pose_sha256=hashlib.sha256((full/'apparent_floor_marker_pose.csv').read_bytes()).hexdigest())
    rows.append(row)
ax.scatter(315,90,marker='^',c='black',label='Planned start315/90mm')
ax.scatter(135,225,marker='x',c='black',s=70,label='Planned Lend135/225mm')
ax.axhline(225,c='black',ls='--',lw=1,label='Planned horizontal exit line')
ax.arrow(315,90,-45,45,head_width=4,color='black',length_includes_head=True)
ax.set(xlim=(0,360),ylim=(360,0),xlabel='Registered board x [mm]',ylabel='Registered board y [mm]',aspect='equal',
    title='Board-fixed apparent midpoint / absolute axle uncalibrated')
ax.set_xticks(np.arange(0,361,45));ax.set_yticks(np.arange(0,361,45));ax.grid(alpha=.3);ax.legend(fontsize=8)
fig.savefig(OUT/'board_fixed_apparent_midpoint.png',dpi=160);plt.close(fig)
result=dict(rows=rows,coordinate_frame='Registered board TL origin; x right,y down; no measured-entry alignment.',
    observed_point='Apparent floor projection of front/rear marker midpoint, not height-corrected absolute axle.',
    limitations=['Camera foot and absolute body/axle geometry remain uncalibrated. Height-dependent absolute offset remains.',
        'Absolute apparent positions must not replace height-corrected relative geometry or be claimed as calibrated axle accuracy.'])
with (OUT/'board_fixed_review.json').open('x',encoding='utf-8') as file:
    json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
