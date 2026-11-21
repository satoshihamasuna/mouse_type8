"""前回の右45°入口の動画軌跡を使い、配置変更後のタイヤと停止位置を照合する。"""
import argparse
import json
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',type=Path)
    parser.add_argument('snapshot',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    stage=args.stage.resolve()
    baseline=stage.parent/'baseline01'
    previous=read(baseline/'static_on02/static_check.json')
    actual=read(args.snapshot)
    setting=read(stage/'candidate.settings.json')
    result=read(baseline/'baseline_review/review.json')
    trace=np.loadtxt(baseline/'baseline_review/observed_route_conservative.csv',delimiter=',',skiprows=1)
    course=np.column_stack([-trace[:,0],trace[:,1]])
    steps=np.diff(course,axis=0)
    progress=np.r_[0,np.cumsum(np.linalg.norm(steps,axis=1))]
    bend=np.clip((progress-(90+setting['parameters']['lstart']))/63,0,1)
    relative_correction=-setting['parameters']['correction']
    shift=actual['heading_deg']-previous['heading_deg']
    heading_estimate=91.32347438215044+shift
    scatter=actual['maximum_heading_deviation_from_median_deg']
    holes=np.array([(x,y) for x in np.arange(0,361,90) for y in np.arange(0,361,90)])
    checks=[]
    for change in (-scatter,0.,scatter):
        heading=heading_estimate+change
        rotation=np.deg2rad(heading-90+relative_correction*bend[1:])
        movement=np.column_stack([np.cos(rotation)*steps[:,0]-np.sin(rotation)*steps[:,1],
                                  np.sin(rotation)*steps[:,0]+np.cos(rotation)*steps[:,1]])
        path=np.array(actual['center_board_mm'])+np.vstack([[0,0],np.cumsum(movement,axis=0)])
        yaw=np.deg2rad(heading+(result['exit_direction_deg']+relative_correction)*bend)
        forward=np.column_stack([np.cos(yaw),np.sin(yaw)])
        left=np.column_stack([-forward[:,1],forward[:,0]])
        contacts=np.stack([path+forward*u+left*v for u,v in ((0,-14),(0,14),(-27,-5),(-27,5),(27,-5),(27,5))])
        gap=np.linalg.norm(contacts[:,:,None,:]-holes[None,None,:,:],axis=3)
        i,j,k=np.unravel_index(gap.argmin(),gap.shape)
        corners=np.stack([path+forward*u+left*v for u,v in ((-31,-20),(-31,20),(31,-20),(31,20))])
        checks.append(dict(entry_heading_est_deg=float(heading),closest_contact_center_to_hole_mm=float(gap[i,j,k]),
            closest_contact=['right_wheel','left_wheel','rear_pad_right','rear_pad_left','front_pad_right','front_pad_left'][i],
            nearest_hole_board_mm=holes[k].tolist(),estimated_stop_board_mm=path[-12].tolist(),
            path_body_extent_board_mm=[corners.min(axis=(0,1)).tolist(),corners.max(axis=(0,1)).tolist()]))
    report=dict(
        source_pose=str(args.snapshot.resolve()),
        position_shift_from_baseline_mm=(np.array(actual['center_board_mm'])-previous['center_board_mm']).tolist(),
        apparent_heading_shift_deg=float(shift),center=checks[1],lower_heading=checks[0],upper_heading=checks[2],
        approximate_minimum_contact_center_distance_mm=min(c['closest_contact_center_to_hole_mm'] for c in checks),
        assumptions='Prior video track, observed brake travel plus 8 mm. Attitude estimate from entry direction and difference in static marking. Tread 28mm, approximate front/rear locations. Use operator confirmation for actual contacts and support.')
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write('\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
