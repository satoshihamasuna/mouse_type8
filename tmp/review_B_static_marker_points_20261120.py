"""保存済みB静止動画の実検出点を固定10フレームで確認する。機器操作なし。"""
from pathlib import Path
import json
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration,board_to_pose_transform,
    colour_components,select_marker_pair,transform_points)

BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
tag=sys.argv[1]
if not tag.replace('_','').isalnum():
    raise ValueError('英数字のタグが必要')
output_tag=sys.argv[3] if len(sys.argv)>3 else tag
if not output_tag.replace('_','').isalnum():
    raise ValueError('英数字の出力タグが必要')
config_path=Path(sys.argv[2]) if len(sys.argv)>2 else BASE/'config.current.json'
config=json.loads(config_path.read_text(encoding='utf-8'))
reference=json.loads(Path(config['reference']).read_text(encoding='utf-8'))
geometry=json.loads(Path(config['geometry']).read_text(encoding='utf-8'))
capture=json.loads((BASE/(tag+'_static_capture.json')).read_text(encoding='utf-8'))
video=cv2.VideoCapture(capture['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
rows=[]
tiles=[]


def catalog(candidates):
    return [dict(xy=p.tolist(),area=int(area)) for p,area in candidates
            if 700<p[0]<780 and 100<p[1]<230]


for requested_pts in np.arange(1.,3.,.2):
    video.set(cv2.CAP_PROP_POS_MSEC,requested_pts*1000)
    ok,frame=video.read()
    assert ok
    pts=video.get(cv2.CAP_PROP_POS_MSEC)/1000
    registration=marker_registration(frame,reference)
    assert registration['available']
    matrix=board_to_pose_transform(reference)@np.array(registration['homography_image_to_board_mm'])
    fronts=colour_components(frame,geometry['front_colour'],geometry['marker_detection'])
    rears=colour_components(frame,geometry['rear_colour'],geometry['marker_detection'])
    pair=select_marker_pair(fronts,rears,matrix,None,
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
        marker_span_limits_mm=geometry['marker_span_limits_mm'])
    assert pair is not None
    pixels=transform_points(pair,np.linalg.inv(matrix))
    span=float(np.linalg.norm(pair[0]-pair[1]))
    rows.append(dict(pts=pts,front_image_px=pixels[0].tolist(),rear_image_px=pixels[1].tolist(),
        span_mm=span,front_candidates=catalog(fronts),rear_candidates=catalog(rears)))
    for point,colour in zip(pixels,[(0,255,0),(255,0,255)]):
        cv2.circle(frame,tuple(np.rint(point).astype(int)),4,colour,1,cv2.LINE_AA)
    left=int(np.clip(pixels[:,0].mean()-37,0,frame.shape[1]-75))
    top=int(np.clip(pixels[:,1].min()-12,0,frame.shape[0]-100))
    tile=cv2.resize(frame[top:top+100,left:left+75],(225,300),interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(tile,(0,0),(225,24),(0,0,0),-1)
    cv2.putText(tile,f'{pts:.3f}s span{span:.1f}',(5,17),
        cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
    tiles.append(tile)
video.release()
cv2.imwrite(str(BASE/(output_tag+'_detection_points.jpg')),
    np.vstack([np.hstack(tiles[:5]),np.hstack(tiles[5:])]))
with (BASE/(output_tag+'_detection_points.json')).open('x',encoding='utf-8') as stream:
    json.dump(dict(front_green_rear_magenta=True,frames=rows),stream,indent=2)
metadata=json.loads(Path(capture['metadata_file']).read_text(encoding='utf-8'))
print(json.dumps(dict(focus_lock=metadata.get('focus_lock'),
    selected_spans_mm=[round(row['span_mm'],2) for row in rows],
    rear_centres=[row['rear_image_px'] for row in rows]),ensure_ascii=False),flush=True)
