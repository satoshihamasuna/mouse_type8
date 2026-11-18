"""実C++ターンEKFを保存済み5msログで因果的に再生し、同じ動画基準と照合する。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np
import pandas as pd
import subprocess

from turn_trajectory_overlay import error_metrics

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/tests'))
from test_turn_pose_firmware import build_harness


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    executable=build_harness(args.output,'A')
    report=json.loads((args.review/'manifest.json').read_text(encoding='utf-8'))
    summary=[]
    for case in report['cases']:
        paths=list(case['inputs_sha256'])
        csv=Path(next(p for p in paths if p.endswith('motion.csv')))
        for name,expected in case['inputs_sha256'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:
                raise ValueError('Input changed: '+name)
        settings=json.loads(csv.with_suffix('.settings.json').read_text(encoding='utf-8'))
        raw=pd.read_csv(csv)
        period=raw['irq.period_us'].median()/1e6
        times=raw.time_ms.to_numpy(float)/1000
        dt=np.r_[np.median(np.diff(times)),np.diff(times)]
        speed=raw['ego.velo'].to_numpy()/(period*1000)
        wheel=(raw['ego.velo']-.010*raw['ego.accel']).to_numpy()/(period*1000)
        omega=raw['ego.rad_velo'].to_numpy()
        target=raw['ideal.velo'].to_numpy()
        command=raw['ideal.rad_velo'].to_numpy()
        # The old stream has no control-state field. Exclude the small straight
        # wall-steering commands; infer the rotation window as in video review.
        turning=abs(command)>.02*max(abs(command))
        moving=abs(target)>.01
        first,last=np.flatnonzero(moving)[[0,-1]]
        heading=np.zeros(len(raw))
        dx=np.zeros(len(raw));dy=np.zeros(len(raw))
        for i in range(first+1,last+1):
            w=.5*(omega[i-1]+omega[i])
            heading[i]=heading[i-1]+w*dt[i]
            angle=.5*(heading[i-1]+heading[i])
            distance=.5*(speed[i-1]+speed[i])*dt[i]*1000
            dx[i]=distance*np.cos(angle);dy[i]=distance*np.sin(angle)
        before=np.r_[0,heading[:-1]]
        pattern=3 if case['motion']=='long_r90' else 5
        rows=np.column_stack([dt,speed,wheel,target,omega,raw['ego.horizon_accel.get'],raw['ego.accel'],before,
                              dy,dx,np.full(len(raw),650 if case['suction'] else 0),np.full(len(raw),pattern),
                              turning,np.full(len(raw),case['suction']),np.ones(len(raw)),
                              (abs(target)<.01)&(abs(command)<.05)])
        run=args.output/case['id'];run.mkdir()
        np.savetxt(run/'input.txt',rows,fmt='%.9g')
        result=subprocess.run([str(executable),str(run/'input.txt'),str(run/'filter.csv')],capture_output=True,text=True,timeout=30)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        filtered=pd.read_csv(run/'filter.csv')
        adopted=np.column_stack([np.where(filtered.applied,filtered.forward_mm,dx),np.where(filtered.applied,filtered.left_mm,dy)])
        simple=np.column_stack([np.cumsum(dx),np.cumsum(dy)])[first:last+1]
        adopted=np.cumsum(adopted,axis=0)[first:last+1]
        log_t=times[first:last+1]-times[first]
        # The HTML manifest records reference transforms and source paths; read
        # the same measured video positions from its embedded payload.
        import re
        html=(args.review/'compare.html').read_text(encoding='utf-8')
        payload=json.loads(re.search(r'const DATA=(.*);\nconst \$',html).group(1))
        data=next(c for c in payload['cases'] if c['id']==case['id'])
        v=np.array([[np.nan if q is None else q for q in r] for r in data['video']],float)
        keep=np.isfinite(v[:,1:]).all(axis=1)&(v[:,0]>=0)&(v[:,0]<=log_t[-1])
        video_xy=np.column_stack([v[keep,2],-v[keep,1]])
        a,_=error_metrics(v[keep,0],video_xy,log_t,simple,data['turn'])
        b,_=error_metrics(v[keep,0],video_xy,log_t,adopted,data['turn'])
        active=turning
        summary.append(dict(trial=case['id'],speed=case['speed'],motion=case['motion'],suction=case['suction'],
                            simple_rmse_mm=a['rmse_mm'],adopted_rmse_mm=b['rmse_mm'],
                            turning_rows=int(active.sum()),applied_rows=int(filtered.loc[active,'applied'].sum()),
                            reasons=filtered.loc[active,'reason'].value_counts().to_dict()))
        pd.DataFrame(dict(time_s=log_t,simple_forward_mm=simple[:,0],simple_left_mm=simple[:,1],
                          adopted_forward_mm=adopted[:,0],adopted_left_mm=adopted[:,1])).to_csv(run/'trajectory.csv',index=False)
    table=pd.DataFrame(summary)
    table.to_csv(args.output/'summary.csv',index=False)
    sources=[ROOT/'Task/Inc/turn_pose_ekf.h',ROOT/'Params/turn_pose_param.h',ROOT/'Module/Src/interrupt.cpp',Path(__file__)]
    (args.output/'report.json').write_text(json.dumps(dict(cases=summary,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        limitation='Causal 5ms replay; omitted 1ms samples and actual MCU runtime are not reconstructed.'),
        ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(table[['trial','simple_rmse_mm','adopted_rmse_mm','applied_rows','turning_rows']].to_string(index=False))
    print(table.groupby(['speed','motion'])[['simple_rmse_mm','adopted_rmse_mm']].median().to_string())


if __name__=='__main__':main()
