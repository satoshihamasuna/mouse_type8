"""保存済みB右V90出口の角速度追従をMCU時刻で比較する。機器操作なし。"""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from video_marker_pose import log_elapsed_seconds

def review(run,output):
    trial=json.loads((run/'trial.json').read_text(encoding='utf-8'))
    settings=json.loads(Path(trial['type8_settings_json']).read_text(encoding='utf-8'))
    assert trial['mouse']==settings['mouse']=='B' and settings['motion_name']=='r_v90'
    path=Path(trial['type8_log_csv']);log=pd.read_csv(path);time=log_elapsed_seconds(log)
    target=log['ideal.rad_velo'].to_numpy();actual=log['ego.rad_velo'].to_numpy()
    mask=abs(target)>.02*max(abs(target));indices=np.flatnonzero(mask)
    before=max(0,indices[0]-4);after=min(len(time),indices[-1]+13)
    result=dict(mouse='B',motion='r_v90',turn_samples=int(mask.sum()),
        omega_rmse_rad_s=float(np.sqrt(np.mean((actual[mask]-target[mask])**2))),
        target_peak_rad_s=float(np.max(abs(target))),actual_peak_rad_s=float(np.max(abs(actual))),
        speed_rmse_m_s=float(np.sqrt(np.mean((log['ego.velo'][mask]-log['ideal.velo'][mask])**2))),
        minimum_battery_V=float(log.Battery.min()),
        target_integrated_turn_deg=float(np.degrees(np.trapezoid(target[before:after],time[before:after]))),
        actual_integrated_turn_deg=float(np.degrees(np.trapezoid(actual[before:after],time[before:after]))),
        scope='MCU time_ms; integration includes 4 samples before and12 after; no camera endpoint claim',
        parameters=settings['parameters'],pid=settings['pid'],feedforward=settings['feedforward'],
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,Path(trial['type8_settings_json']),run/'trial.json',Path(__file__)]})
    output.mkdir(exist_ok=False)
    (output/'control.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    fig,ax=plt.subplots(figsize=(8,3.5),layout='constrained')
    ax.plot(time[before:after]-time[indices[0]],target[before:after],label='Target')
    ax.plot(time[before:after]-time[indices[0]],actual[before:after],label='B measured')
    ax.set(xlabel='Time from turn start [s]',ylabel='Angular velocity [rad/s]',title=f"B out_R135: OM P={settings['pid']['om_kp']:.2f}")
    ax.grid(alpha=.3);ax.legend();fig.savefig(output/'angular_tracking.png',dpi=140);plt.close(fig)
    print(json.dumps({k:result[k] for k in ('omega_rmse_rad_s','target_peak_rad_s','actual_peak_rad_s','speed_rmse_m_s','minimum_battery_V')}))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();review(args.run.resolve(),args.output.resolve())
