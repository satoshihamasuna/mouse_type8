"""B180度の保存済みOFF試写の色マスクと静止ばらつきを診断。実機操作なし。"""
from pathlib import Path
import copy
import argparse
import json
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration,registration_quad,pose_world_corners,
    board_to_pose_transform,colour_components,select_marker_pair,transform_points)
from marker_detection import colour_mask
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
parser=argparse.ArgumentParser()
parser.add_argument('--output',default='segmentation_diagnostic_v1')
parser.add_argument('--saturations',type=int,nargs='+',default=[25,20,15])
parser.add_argument('--hue-max',type=int,default=95)
parser.add_argument('--plan',type=Path)
args=parser.parse_args()
assert args.output.replace('_','').isalnum()
OUT=(args.plan.parent if args.plan else BASE/'off_static01')/args.output
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
config=read(args.plan) if args.plan else read(BASE/'baseline01/config.json')
reference,geometry=read(config['reference']),read(config['geometry'])
capture=dict(video_file=config['video']) if args.plan else read(BASE/'off_static01/capture.json')
video=cv2.VideoCapture(capture['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
rows=[];tiles=[]
for requested in np.linspace(3.3,4.9,17):
    video.set(cv2.CAP_PROP_POS_MSEC,float(requested*1000))
    ok,frame=video.read();assert ok
    pts=video.get(cv2.CAP_PROP_POS_MSEC)/1000
    registration=marker_registration(frame,reference)
    quad=registration_quad(registration)
    assert quad is not None and len(registration['detected_marker_ids'])>=2
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    variants=[]
    for saturation in args.saturations:
        profile=copy.deepcopy(geometry['marker_detection'])
        profile['green']['minimum_saturation']=saturation
        profile['green']['hue_ranges']=[[65,args.hue_max]]
        fronts=colour_components(frame,'green',profile)
        rears=colour_components(frame,'red',profile)
        pair=select_marker_pair(fronts,rears,matrix,None,
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
            marker_span_limits_mm=geometry['marker_span_limits_mm'])
        if pair is None:
            variants.append(dict(saturation=saturation,detected=False));continue
        pixels=transform_points(pair,np.linalg.inv(matrix))
        board=transform_points(pair,np.linalg.inv(board_to_pose_transform(reference)))
        delta=board[0]-board[1]
        variants.append(dict(saturation=saturation,detected=True,
            front_image_px=pixels[0].tolist(),rear_image_px=pixels[1].tolist(),
            heading_deg=float(np.degrees(np.arctan2(delta[1],delta[0]))),
            front_candidates=[dict(xy=p.tolist(),area=int(a)) for p,a in fronts
                if np.linalg.norm(p-pixels[0])<20],
            rear_candidates=[dict(xy=p.tolist(),area=int(a)) for p,a in rears
                if np.linalg.norm(p-pixels[1])<20]))
    rows.append(dict(pts=pts,variants=variants))
    # 先に固定した画角の車体ROI。姿勢評価や合格判定の条件には使わない。
    crop=frame[215:311,715:775]
    marked=crop.copy()
    for variant,colour in zip(variants,[(255,0,255),(0,255,255),(255,255,0)]):
        if variant.get('detected'):
            for key in ('front_image_px','rear_image_px'):
                point=np.array(variant[key])-np.array([715,215])
                cv2.circle(marked,tuple(np.rint(point).astype(int)),2,colour,1)
    mask_profile=copy.deepcopy(geometry['marker_detection'])
    mask_profile['green'].update(minimum_saturation=args.saturations[0],hue_ranges=[[65,args.hue_max]])
    selected_mask,_=colour_mask(hsv,'green',mask_profile)
    tile=np.hstack([marked,cv2.cvtColor(selected_mask[215:311,715:775],cv2.COLOR_GRAY2BGR)])
    tile=cv2.resize(tile,(240,192),interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(tile,(0,0),(240,20),(0,0,0),-1)
    label='/'.join(str(s) for s in args.saturations)
    cv2.putText(tile,f'{pts:.3f}s S{label} H65..{args.hue_max}',(3,14),cv2.FONT_HERSHEY_SIMPLEX,.33,(255,255,255),1)
    tiles.append(tile)
video.release()
summaries=[]
for saturation in args.saturations:
    selected=[v for row in rows for v in row['variants'] if v['saturation']==saturation and v.get('detected')]
    if not selected:
        summaries.append(dict(saturation=saturation,observed=0));continue
    heading=np.array([v['heading_deg'] for v in selected])
    front=np.array([v['front_image_px'] for v in selected])
    rear=np.array([v['rear_image_px'] for v in selected])
    summaries.append(dict(saturation=saturation,observed=len(selected),
        heading_median_deg=float(np.median(heading)),
        heading_rms_deg=float(np.sqrt(np.mean((heading-np.median(heading))**2))),
        front_xy_std_px=np.std(front,axis=0).tolist(),rear_xy_std_px=np.std(rear,axis=0).tolist(),
        front_xy_range_px=np.ptp(front,axis=0).tolist(),rear_xy_range_px=np.ptp(rear,axis=0).tolist()))
while len(tiles)%6:tiles.append(np.zeros_like(tiles[0]))
assert cv2.imwrite(str(OUT/'marker_points_and_mask.jpg'),np.vstack([np.hstack(tiles[i:i+6]) for i in range(0,len(tiles),6)]))
with (OUT/'report.json').open('x',encoding='utf-8') as output:
    json.dump(dict(scope='事前指定の17点による診断。元の静止不合格は保持。新しい設定の採用や独立検証の代用にしない。',
        source_capture=str(BASE/'off_static01/capture.json'),hue_range=[65,args.hue_max],summaries=summaries,rows=rows),output,ensure_ascii=False,indent=2)
print(json.dumps(summaries,ensure_ascii=False),flush=True)
