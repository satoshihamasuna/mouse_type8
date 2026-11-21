"""同設定R180の旋回速度を比較する。原本と採用値は変更しない。"""
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
NAMES=('resume_20261121_Le40p5_retry01','lend31p5_r47p6_c1p5_charged_01','lend36p0_r47p6_c1p5_01')
out=BASE/NAMES[-1]/'speed_variation_audit_v1'
assert not out.exists()
out.mkdir()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
rows=[];hashes={};figure,axes=plt.subplots(3,1,figsize=(10,10),layout='constrained')
for ax,name in zip(axes,NAMES):
    stage=BASE/name
    run=next((stage/'local_execution').glob('run_*/saved_1ms.csv')).parent
    settings=read(run/'saved_1ms.settings.json')
    assert settings['mouse']=='A' and settings['motion_name']=='long_r180'
    assert settings['parameters']['r_min']==-47.6 and settings['parameters']['correction']==1.5
    review=read(stage/'entry_clock_midpoint_review_v1/report.json')
    assert review['relative_geometry_eligible'] and all(review['checks'].values())
    raw=pd.read_csv(run/'saved_1ms.csv')
    assert np.array_equal(raw.cnt,np.arange(len(raw)))
    step=(raw['Encoder_GetProperty_Right().sp_pulse']-raw['Encoder_GetProperty_Left().sp_pulse'])*15.02*np.pi/172/2
    encoder_nominal=step.rolling(20,min_periods=20).mean().to_numpy()
    anticipatory=.010*raw['ego.accel'].to_numpy()
    formula=encoder_nominal+anticipatory
    mismatch=raw['ego.velo'].to_numpy()-formula
    mask=abs(raw['ideal.rad_velo'].to_numpy())>=abs(raw['ideal.rad_velo']).max()*.02
    idx=np.flatnonzero(mask)
    tick=review['timing']['measured_tick_ms']/1000
    zero=review['timing']['saved_zero_video_s']
    time=zero+raw.cnt.to_numpy()*tick
    pose=pd.read_csv(stage/'entry_clock_midpoint_review_v1/relative_trajectory.csv')
    valid=pose.detected.astype(bool).to_numpy()
    t=pose.loc[valid,'time_video_s'].to_numpy()
    xy=pose.loc[valid,['right_mm','forward_mm']].to_numpy()
    stamps=[];speeds=[]
    # 実観測の時刻と位置へ局所二次式を当てる。欠測点は生成しない。
    for i,stamp in enumerate(t):
        if not time[idx[0]]<=stamp<=time[idx[-1]]:continue
        local=abs(t-stamp)<=.0126
        if local.sum()<5 or np.diff(t[local]).max()>.010:continue
        coef=np.polynomial.polynomial.polyfit(t[local]-stamp,xy[local],2)
        stamps.append(stamp);speeds.append(float(np.linalg.norm(coef[1])/1000))
    assert len(speeds)>=15
    row=dict(stage=name,parameters=settings['parameters'],battery_min_v=float(raw.Battery.min()),
        ideal_speed_mean_m_s=float(raw.loc[mask,'ideal.velo'].mean()),
        ego_speed_mean_m_s=float(raw.loc[mask,'ego.velo'].mean()),
        encoder20_nominal_mean_m_s=float(encoder_nominal[mask].mean()),
        encoder20_measured_tick_mean_m_s=float((encoder_nominal[mask]/(tick*1000)).mean()),
        imu_anticipatory_mean_m_s=float(anticipatory[mask].mean()),
        ego_formula_rmse_m_s=float(np.sqrt(np.nanmean(mismatch[mask]**2))),
        single_tick_pulse_mean_m_s=float((step.to_numpy()[mask]/(tick*1000)).mean()),
        relative_midpoint_video_speed_mean_m_s=float(np.mean(speeds)),
        video_speed_observations=len(speeds),width45_mm=review['return_spacing_at_forward45_mm'],
        path_angle_deg=review['path_angle_deg'],lend_right_forward_mm=review['lend_right_forward_mm'],
        source_formula_matches_within_2mm_s=bool(np.sqrt(np.nanmean(mismatch[mask]**2))<=.002))
    rows.append(row)
    trace=pd.DataFrame(dict(cnt=raw.cnt,time_video_s=time,turn=mask,
        ego_m_s=raw['ego.velo'],encoder20_nominal_m_s=encoder_nominal,
        imu_anticipatory_m_s=anticipatory,formula_m_s=formula,formula_error_m_s=mismatch))
    trace.to_csv(out/(name+'_signals.csv'),index=False)
    pd.DataFrame(dict(time_video_s=stamps,relative_midpoint_speed_m_s=speeds)).to_csv(out/(name+'_video_speed.csv'),index=False)
    phase=(time-time[idx[0]])/(time[idx[-1]]-time[idx[0]])
    ax.plot(phase[mask],raw.loc[mask,'ego.velo'],label='Control estimate')
    ax.plot(phase[mask],encoder_nominal[mask]/(tick*1000),label='20-sample wheel speed, measured tick')
    ax.plot((np.array(stamps)-time[idx[0]])/(time[idx[-1]]-time[idx[0]]),speeds,'.-',label='Relative midpoint video derivative')
    ax.set(title=f"{name}: width {row['width45_mm']:.2f} mm",ylabel='Speed [m/s]',xlabel='Angular profile phase')
    ax.grid();ax.legend()
    for p in (run/'saved_1ms.csv',run/'saved_1ms.settings.json',stage/'entry_clock_midpoint_review_v1/report.json',stage/'entry_clock_midpoint_review_v1/relative_trajectory.csv'):
        hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
source=[Path(__file__),ROOT/'Module/Src/interrupt.cpp',ROOT/'Peripheral/Src/encoder.c',ROOT/'Params/mouse_config.h',ROOT/'Task/Src/ctrl_task.cpp',ROOT/'Params/turn_pose_param.h',ROOT/'Task/Inc/turn_pose_ekf.h']
hashes.update({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source})
report=dict(status='OBSERVATIONAL_SPEED_VARIATION_AUDIT',trials=rows,
    speed_formula_in_current_source='ego.velo = encoder20_nominal_m_s + 0.010 * ego.accel',
    current_source_turn_ekf_eligibility='calibrated suction speed 1.6 +/-0.05 m/s target; 1.8 m/s target excluded. Runtime mode alone does not prove application.',
    source_formula_compatible_with_logs=all(r['source_formula_matches_within_2mm_s'] for r in rows),
    observations_imputed=False,parameter_adoption_allowed=False,firmware_changed=False,
    causal_conclusion=False,limitations=[
        'Battery, entry state, inferred command radius and physical response covary; battery causation not isolated.',
        'Video velocity is a local derivative of registered relative marker midpoint, not calibrated axle ground speed.',
        '20-sample wheel mean and IMU anticipation are source-derived signals; logged single pulses have quantization.',
        'Current-source eligibility is not a captured runtime application history or firmware-image identity.',
        'No new FF, EKF, estimator or geometry value is adopted from this comparison.'],source_sha256=hashes)
with (out/'report.json').open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
pd.DataFrame([{k:v for k,v in r.items() if not isinstance(v,(dict,list))} for r in rows]).to_csv(out/'summary.csv',index=False)
figure.savefig(out/'speed_comparison.png',dpi=150);plt.close(figure)
print(json.dumps(dict(output=str(out),compatible=report['source_formula_compatible_with_logs'],trials=rows)),flush=True)
