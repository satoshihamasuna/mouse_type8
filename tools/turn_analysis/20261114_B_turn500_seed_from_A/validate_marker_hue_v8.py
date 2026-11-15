"""固定した色判定を、調整に使っていないBの保存映像で照合する。機器操作なし。"""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import read_video_pose

def load(p): return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('case',choices=['static','outR45','inR135']); a=p.parse_args()
    protocol_path=BASE/'marker_hue_revision_v8/frozen_protocol.json'; protocol=load(protocol_path)
    case=protocol['cases'][a.case]; out=BASE/'marker_hue_revision_v8'/a.case; out.mkdir(exist_ok=False)
    geometry_path=Path(protocol['geometry']); geometry=load(geometry_path)
    reference_path=Path(protocol['reference']); reference=load(reference_path)
    video_path=Path(case['video']); old_path=Path(case['old_pose'])
    pose,_,_,_=read_video_pose(video_path,front_colour='yellow',rear_colour='red',marker_reference=reference,
        marker_detection=geometry['marker_detection'],nominal_marker_span_mm=50,marker_span_limits_mm=[40,65],
        video_window=case['window_s'],frame_stride=1)
    pose.to_csv(out/'pose.csv',index=False)
    good=pose.marker_detected_raw & pose.calibration_valid
    old=pd.read_csv(old_path); shared=pose.merge(old.loc[old.marker_detected_raw & old.calibration_valid],on='frame',suffixes=('_new','_old'))
    shared=shared[shared.marker_detected_raw_new & shared.calibration_valid_new]
    assert len(shared)>=.8*len(pose)
    centre=np.linalg.norm(shared[['center_x_mm_new','center_y_mm_new']].to_numpy()-shared[['center_x_mm_old','center_y_mm_old']].to_numpy(),axis=1)
    heading=(np.degrees(shared.heading_rad_new-shared.heading_rad_old)+180)%360-180
    points=pose.loc[good,['center_x_mm','center_y_mm']].to_numpy(); median=np.median(points,axis=0); dist=np.linalg.norm(points-median,axis=1)
    result=dict(case=a.case,frames=len(pose),observation_rate=float(good.mean()),shared_frames=len(shared),
        centre_change_median_mm=float(np.median(centre)),centre_change_p95_mm=float(np.percentile(centre,95)),centre_change_max_mm=float(centre.max()),
        heading_change_max_deg=float(abs(heading).max()),stationary_scatter_rms_mm=float(np.sqrt(np.mean(dist**2))) if a.case=='static' else None,
        stationary_max_deviation_mm=float(dist.max()) if a.case=='static' else None,
        frozen_protocol=str(protocol_path),input_sha256={str(p):sha(p) for p in [video_path,old_path,geometry_path,reference_path,protocol_path,Path(__file__),ROOT/'tools/analysis/video_marker_pose.py']})
    passed=bool(good.mean()>=case['minimum_observation_rate'] and np.median(centre)<=.75 and np.percentile(centre,95)<=1.5 and centre.max()<=2.5 and abs(heading).max()<=3)
    if a.case=='static': passed=passed and len(pose)>=280 and result['stationary_scatter_rms_mm']<=.5 and dist.max()<=1.5
    result['state']='PASSED' if passed else 'FAILED'
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='input_sha256'}),flush=True)
    assert passed

if __name__=='__main__': main()
