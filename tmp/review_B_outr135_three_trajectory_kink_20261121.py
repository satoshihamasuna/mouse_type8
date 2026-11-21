"""図で気になった旋回開始直後の実マーカー位置を原動画で照合。元試行は変更しない。"""
from pathlib import Path
import json
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
run=base/'standard_flow_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_standard_green_20261121_001'
out=base/'three_trajectory_kink_review_v1';assert not out.exists();out.mkdir()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
relative=pd.read_csv(run/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
selected=relative[(relative.forward_mm.between(135,185))&(relative.right_mm< -2)&relative.registered_midpoint_valid.astype(bool)]
assert len(selected)>0
indices=sorted(set(index for selected_index in selected.index for index in range(max(0,selected_index-1),min(len(relative),selected_index+2))))
cap=cv2.VideoCapture(read(run/'trial.json')['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
fig,axes=plt.subplots(3,4,figsize=(11,8.5),layout='constrained');records=[]
assert len(indices)<=12
for ax,index in zip(axes.flat,indices):
    row=pose.iloc[index];local=relative.iloc[index]
    cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
    front=np.array([row.front_image_x_px,row.front_image_y_px]);center=np.array([row.midpoint_image_x_px,row.midpoint_image_y_px]);rear=2*center-front
    canvas=frame.copy()
    for point,colour in ((front,(0,255,0)),(rear,(0,0,255))):cv2.circle(canvas,tuple(np.rint(point).astype(int)),6,colour,1)
    x,y=np.rint(center).astype(int)
    ax.imshow(cv2.cvtColor(canvas[max(0,y-70):y+70,max(0,x-70):x+70],cv2.COLOR_BGR2RGB))
    ax.set_title(f'{int(row.frame)} / {row.time_video_s:.4f}s\nx={local.right_mm:.2f} y={local.forward_mm:.2f}',fontsize=8);ax.axis('off')
    records.append(dict(frame=int(row.frame),time_video_s=float(row.time_video_s),right_forward_mm=[float(local.right_mm),float(local.forward_mm)],
        raw_observed=bool(row.marker_detected_raw),front_px=front.tolist(),rear_px=rear.tolist()))
for ax in axes.flat[len(indices):]:ax.axis('off')
cap.release();fig.savefig(out/'actual_pairs_at_kink.jpg',dpi=145);plt.close(fig)
with (out/'diagnostic.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(state='VISUAL_REVIEW_PENDING',rows=records,source_trial=str(run/'trial.json'),
        original_sources_modified=False,hardware_operations=0),stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(samples=len(records),rows=records),ensure_ascii=False),flush=True)
