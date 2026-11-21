"""保存動画の実マーカーを目視照合するための画像と方向窓感度を保存する。実機操作なし。"""
from pathlib import Path
import importlib.util
import json
import sys

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1/next_correction_plus1p0_20261120_v1/run_session01/run_B_R90_2000_correction1p0_20261120_001'
SNAP = RUN/'analysis_source_used'
sys.path[:0] = [str(SNAP), str(ROOT/'tools'), str(ROOT/'tools/analysis')]
for name in ('pixel_camera_geometry', 'pixel_camera_reference', 'marker_detection', 'body_appearance_model', 'video_marker_pose'):
    spec = importlib.util.spec_from_file_location(name, SNAP/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
from video_marker_pose import iter_video_frames

report = json.loads((RUN/'full/report.json').read_text(encoding='utf-8'))
geometry = json.loads((RUN/'geometry_review_v1/video_comparison.json').read_text(encoding='utf-8'))['comparisons'][0]
pose = pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
relative = pd.read_csv(RUN/'full/relative_marker_midpoint.csv')
valid = pose.marker_detected_raw & pose.calibration_valid
assert len(pose) == len(relative)
time = pose.time_video_s.to_numpy()
start, end = report['required_motion_window_quality']['required_interval_pts_s']
indices = set()
for stamp in np.linspace(start, end, 24):
    indices.add(int(np.argmin(np.abs(time-stamp))))
turn = (time >= geometry['entry_window_pts_s'][1]) & (time <= geometry['exit_window_pts_s'][0])
indices.update(np.flatnonzero(turn).tolist())
for stamp in np.linspace(*geometry['stationary_window_pts_s'], 8):
    indices.add(int(np.argmin(np.abs(time-stamp))))
selected = pose.iloc[sorted(indices)]
assert valid.iloc[sorted(indices)].all()
out = RUN/'marker_identity_review_v2'
out.mkdir(exist_ok=False)
points = []
wanted = {}
for row in selected.itertuples():
    front = np.array([row.front_image_x_px, row.front_image_y_px])
    rear = 2*np.array([row.midpoint_image_x_px, row.midpoint_image_y_px])-front
    point = dict(frame=int(row.frame), pts_s=float(row.time_video_s), observed=bool(row.marker_detected_raw), front_px=front.tolist(), rear_px=rear.tolist())
    points.append(point)
    wanted[int(row.frame)] = point
tiles = []
for frame_id, image, pts in iter_video_frames(Path(report['video_file']), raw_orientation=True, video_window=(float(selected.time_video_s.min())-.01,float(selected.time_video_s.max())+.01)):
    if frame_id not in wanted:
        continue
    point = wanted[frame_id]
    front, rear = (np.array(point[k]) for k in ('front_px', 'rear_px'))
    center = np.rint((front+rear)/2).astype(int)
    left = max(0, min(image.shape[1]-150, int(center[0])-75))
    top = max(0, min(image.shape[0]-150, int(center[1])-75))
    for coordinate, colour in ((front,(0,255,0)),(rear,(0,0,255))):
        xy = tuple(np.rint(coordinate).astype(int))
        cv2.circle(image,xy,4,colour,1,cv2.LINE_AA)
        cv2.drawMarker(image,xy,colour,cv2.MARKER_CROSS,3,1,cv2.LINE_AA)
    tile = cv2.resize(image[top:top+150,left:left+150],(300,300),interpolation=cv2.INTER_NEAREST)
    label = np.zeros((30,300,3),np.uint8)
    cv2.putText(label,f"frame {frame_id}  {pts:.3f}s",(6,21),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
    tiles.append(np.vstack([label,tile]))
assert len(tiles) == len(points)
for page_start in range(0,len(tiles),16):
    page = np.zeros((330*4,300*4,3),np.uint8)
    for index,tile in enumerate(tiles[page_start:page_start+16]):
        y,x = divmod(index,4)
        page[y*330:(y+1)*330,x*300:(x+1)*300] = tile
    assert cv2.imwrite(str(out/f'sample_page{page_start//16+1}.png'),page)
(out/'sampled_points.json').write_text(json.dumps(points,indent=2)+'\n',encoding='utf-8')

xy = relative[['center_dx_mm','center_dy_mm']].to_numpy()
def direction(window, shift):
    mask = valid.to_numpy() & (time>=window[0]+shift) & (time<=window[1]+shift)
    pts = xy[mask]
    assert len(pts)>=15
    axis = np.linalg.svd(pts-pts.mean(axis=0),full_matrices=False)[2][0]
    if axis @ (pts[-1]-pts[0])<0:
        axis = -axis
    assert np.ptp(pts @ axis)>=30
    return np.arctan2(axis[1],axis[0])
sensitivity=[]
for shift in (-.005,0,.005):
    entry = direction(geometry['entry_window_pts_s'],shift)
    exit_ = direction(geometry['exit_window_pts_s'],shift)
    angle = np.degrees((exit_-entry+np.pi)%(2*np.pi)-np.pi)
    sensitivity.append(dict(window_shift_s=shift,path_rotation_deg=float(angle)))
(RUN/'direction_window_sensitivity.json').write_text(json.dumps(sensitivity,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(samples=len(points),pages=(len(points)+15)//16,whole_turn_frames=int(turn.sum()),direction_window_sensitivity=sensitivity),indent=2),flush=True)
