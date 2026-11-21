"""B1600右135度入口の角度比較と次の幾何候補を保存する。実機操作なし。"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
OUT=BASE/'angle_comparison_v1'
assert not OUT.exists();OUT.mkdir()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
runs=[('初回：補正＋2.10°',BASE/'baseline01/local_execution/run_B_INR135_1600_baseline_green_20261120_002/recovered_metadata_v1'),
      ('比較：補正−0.40°',BASE/'angle_cm0p4_01/local_execution/run_B_INR135_1600_cm0p4_green_20261120_001')]
rows=[]
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(7,7.5),layout='constrained')
for label,run in runs:
    folder=run/'midpoint_raw_wheel_clock_v1'
    geometry=read(folder/'midpoint_review.json');settings=read(run/'saved_1ms.settings.json')
    assert geometry['independent_optical_geometry_eligible'] and settings['mouse']=='B'
    frame=pd.read_csv(folder/'midpoint_relative_trajectory.csv')
    quality=geometry['independent_clock_required_motion_quality']
    mask=(frame.time_video_s>=quality['video_motion_start_s']-.02)&(frame.time_video_s<=quality['video_motion_end_s']+.05)
    xy=np.where(frame.registered_midpoint_valid.to_numpy(bool)[:,None],frame[['right_mm','forward_mm']],np.nan)
    curve,=ax.plot(*xy[mask].T,label=label)
    ax.scatter(*geometry['midpoint_Lend_right_forward_mm'],color=curve.get_color(),s=45)
    rows.append(dict(label=label,run=str(run),settings=settings,geometry=geometry))
for key in ('pid','feedforward','suction'):
    assert rows[0]['settings'][key]==rows[1]['settings'][key]
for key in rows[0]['settings']['parameters']:
    if key!='correction':assert rows[0]['settings']['parameters'][key]==rows[1]['settings']['parameters'][key]
angle=np.array([row['geometry']['exit_path_direction_change_deg'] for row in rows])
correction=np.array([row['settings']['parameters']['correction'] for row in rows])
assert angle[0]<135<angle[1]
fraction=float((135-angle[0])/(angle[1]-angle[0]))
interpolated_correction=float(correction[0]+fraction*(correction[1]-correction[0]))
point=np.array([row['geometry']['midpoint_Lend_right_forward_mm'] for row in rows])
predicted_point=point[0]+fraction*(point[1]-point[0])
exit_direction=np.array([np.sin(np.deg2rad(135.)),np.cos(np.deg2rad(135.))])
delta_le=float((90-predicted_point[0])/exit_direction[0])
delta_ls=float(135-predicted_point[1]-delta_le*exit_direction[1])
old=rows[1]['settings']['parameters']
candidate=dict(correction=.4,lstart=10.5,lend=17.2,r_min=old['r_min'])
report=dict(mouse='B',motion='in_r135',preset_mm_s=1600,coordinate_basis='height-corrected relative marker midpoint and measured entry direction',
    runs=rows,angle_change_deg=float(angle[1]-angle[0]),changed_correction_deg=float(correction[1]-correction[0]),
    interpolation=dict(fraction=fraction,correction_deg=interpolated_correction,Lend_at_135_right_forward_mm=predicted_point.tolist(),
        delta_Lstart_mm=delta_ls,delta_Lend_mm=delta_le,lstart_mm=old['lstart']+delta_ls,lend_mm=old['lend']+delta_le),
    next_candidate=candidate,next_candidate_executed=False,source_params_changed=False,hardware_operations=0,
    candidate_status='UNVERIFIED_INTERPOLATED_COMBINED_GEOMETRY_CANDIDATE',
    limitations=['角度2点の補間と小さい幾何変更の近似。次の実走で角度・Lend位置を再測定する。',
        'PID/FF/吸引・半径は維持。EKF係数を今回の値として扱わない。',
        '入口相対の単体候補。固定盤面の設置・接続、車軸校正、独立反復と連続接続は別。',
        '両方の既定summaryと自動停止判定は変更しない。撮影停止の静止証拠、後半の動画静止と操作者確認を別記録。'])
with (OUT/'comparison.json').open('x',encoding='utf-8') as file:json.dump(report,file,ensure_ascii=False,indent=2,allow_nan=False)
ax.plot([90,180],[135,45],'--',c='gray',label='目標出口線')
ax.scatter(90,135,c='black',marker='x',s=60,label='Lend目標 (90,135) mm')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',
    title=f'機体B1600 右135度入口\n＋2.10°: {angle[0]:.2f}° / −0.40°: {angle[1]:.2f}°')
ax.grid(alpha=.3);ax.legend(fontsize=9);fig.savefig(OUT/'comparison.png',dpi=145);plt.close(fig)
print(json.dumps(dict(angles_deg=angle.tolist(),interpolation=report['interpolation'],next_candidate=candidate),ensure_ascii=False),flush=True)
