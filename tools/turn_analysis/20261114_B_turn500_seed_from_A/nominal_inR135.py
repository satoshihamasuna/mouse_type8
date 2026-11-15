"""右135°の名目経路。ファームのテーブルを使い、滑り・制御誤差は含めない。"""
from pathlib import Path
import math, re
import numpy as np
import pandas as pd

def make_path(parameters, pre_mm, post_mm):
    root=Path(__file__).resolve().parents[3]
    source=(root/'Params/turn_table.h').read_text(encoding='utf-8')
    body=re.search(r'accel_table\[1001\]\s*=\s*\{(.*?)\};',source,re.S).group(1)
    table=np.array([float(x.strip().rstrip('f')) for x in body.split(',') if x.strip()])
    assert len(table)==1001 and np.isfinite(table).all()
    integral=float(re.search(r'accel_Integral\s*=\s*([0-9.]+)',source).group(1))
    assert parameters['r_min']<0 and parameters['degree']+parameters['correction']<0
    v=parameters['velo'];maximum=v/abs(parameters['r_min']/1000)
    duration=abs(math.radians(parameters['degree']+parameters['correction']))/(integral*maximum)
    pre=pre_mm+parameters['lstart'];post=post_mm+parameters['lend'];rows=[]
    for distance in np.linspace(0,pre,int(math.ceil(pre))+1):
        rows.append(dict(right_mm=0.,forward_mm=float(distance),heading_right_positive_rad=0.,stage='entry'))
    r=0.;f=pre;theta=0.;elapsed=0.
    while elapsed<duration:
        dt=min(.001,duration-elapsed)
        omega=maximum*float(np.interp(elapsed/duration*1000,np.arange(1001),table))
        theta+=omega*dt;r+=v*1000*math.sin(theta)*dt;f+=v*1000*math.cos(theta)*dt
        rows.append(dict(right_mm=r,forward_mm=f,heading_right_positive_rad=theta,stage='turn'))
        elapsed+=dt
    for distance in np.linspace(0,post,int(math.ceil(post))+1)[1:]:
        rows.append(dict(right_mm=r+float(distance)*math.sin(theta),forward_mm=f+float(distance)*math.cos(theta),
            heading_right_positive_rad=theta,stage='exit'))
    return pd.DataFrame(rows),duration
