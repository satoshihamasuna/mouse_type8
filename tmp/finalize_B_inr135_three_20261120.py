"""B1600右135度入口の同一候補3本を保存して出口へ引き継ぐ。実機操作なし。"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
OUT=BASE/'same_candidate_three_review_v1'
assert not OUT.exists();OUT.mkdir()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
specs=[('geometry_c0p4_Ls10p5_Le17p2_02','B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002','midpoint_raw_wheel_clock_v1'),
    ('geometry_c0p4_Ls10p5_Le17p2_repeat_01','B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_001','midpoint_raw_wheel_clock_v1'),
    ('geometry_c0p4_Ls10p5_Le17p2_repeat_02','B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_002','midpoint_raw_wheel_clock_fallback_v2')]
rows=[];plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(7,7.5),layout='constrained')
for i,(stage,rid,analysis) in enumerate(specs,1):
    session=BASE/stage/'local_execution';run=session/('run_'+rid)
    settings=read(run/'saved_1ms.settings.json');geometry=read(run/analysis/'midpoint_review.json')
    operator=read(session/'operator_results'/f'{rid}.json');tracking=read(run/'tracking_1ms_review_v3/verification.json')
    assert geometry['independent_optical_geometry_eligible'] and operator['both_powered_off'] and not operator['contact_detected']
    if rows:
        for key in ('parameters','pid','feedforward','suction','mouse','motion_name','preset_speed_mm_s'):
            assert settings[key]==rows[0]['settings'][key],key
    trajectory=pd.read_csv(run/analysis/'midpoint_relative_trajectory.csv')
    quality=geometry['independent_clock_required_motion_quality']
    mask=trajectory.time_video_s.between(quality['video_motion_start_s']-.02,quality['video_motion_end_s']+.05)
    xy=np.where(trajectory.registered_midpoint_valid.to_numpy(bool)[:,None],trajectory[['right_mm','forward_mm']].to_numpy(),np.nan)
    curve,=ax.plot(*xy[mask].T,label=f'{i}本目 {geometry["exit_path_direction_change_deg"]:.2f}°')
    ax.scatter(*geometry['midpoint_Lend_right_forward_mm'],c=curve.get_color(),s=40)
    rows.append(dict(request_id=rid,run=str(run),settings=settings,geometry=geometry,operator_result=operator,tracking=tracking,
        recording_stop=read(run/'trial.json')['recording_stop'],original_summary=read(run/'full/summary.json')))
angles=np.array([r['geometry']['exit_path_direction_change_deg'] for r in rows])
points=np.array([r['geometry']['midpoint_Lend_right_forward_mm'] for r in rows])
errors=points-[90.,135.]
result=dict(state='THREE_INDEPENDENT_RELATIVE_SINGLE_TURN_CANDIDATE_RUNS_REVIEWED',mouse='B',motion='in_r135',preset_mm_s=1600,
    settings=rows[0]['settings'],runs=rows,valid_same_candidate_runs=3,exit_rotation_deg=dict(values=angles.tolist(),mean=float(angles.mean()),
        min=float(angles.min()),max=float(angles.max()),range=float(np.ptp(angles)),maximum_error_from135_deg=float(np.abs(angles-135).max())),
    Lend=dict(target_right_forward_mm=[90.,135.],values_right_forward_mm=points.tolist(),mean_right_forward_mm=points.mean(0).tolist(),
        mean_residual_mm=errors.mean(0).tolist(),each_residual_norm_mm=np.linalg.norm(errors,axis=1).tolist(),maximum_residual_mm=float(np.linalg.norm(errors,axis=1).max())),
    all_contacts_absent=True,source_params_changed=False,firmware_write_performed=False,execution_resend_count=0,
    next_motion='out_r135',next_folder=str(ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'),
    turn_EKF='Bの幾何/校正と解析対応が未完了のため係数未同定/未反映。',
    limitations=['入口相対のマーカー中点。固定盤面、車軸/機首の絶対校正、連続接続の合格ではない。',
        '1本目は充電前最低7.270V、2/3本目は充電後最低7.953/7.938V。条件差を保持。',
        '3本目はstale_csvにより録画上限へ退避した元結果を保持。全2696フレームを解析し、末尾静止窓と連続CSVの静止証拠を別評価。',
        '元summaryのauto_stop未合格は変更せず、撮影制御、後半動画静止と操作者の停止/接触なし/OFF確認を区別。'])
with (OUT/'review.json').open('x',encoding='utf-8') as file:json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False)
ax.plot([90,180],[135,45],'--',c='gray',label='目標出口線');ax.scatter(90,135,c='black',marker='x',s=60,label='Lend目標')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='機体B1600 右135度入口：同一候補3本\n補正+0.40° / Lstart10.5mm / Lend17.2mm')
ax.grid(alpha=.3);ax.legend(fontsize=8);fig.savefig(OUT/'three_trajectories.png',dpi=145);plt.close(fig)
(OUT/'README.md').write_text(f'機体B1600右135度入口、補正+0.40度・Lstart10.5/Lend17.2mm・半径-42.5mm・吸引650の独立3本。\n\n'
    f'回転量{angles.min():.3f}〜{angles.max():.3f}度、平均{angles.mean():.3f}度、最大目標差{abs(angles-135).max():.3f}度。'
    f'Lend目標90/135mmへの距離誤差は最大{np.linalg.norm(errors,axis=1).max():.3f}mm。全3本で操作者が接触なし・両方OFFを確認。\n\n'
    '単体相対候補として保存。Paramsとファームは未反映。盤面固定・車軸絶対精度・連続接続・BのEKF校正は未確認。'
    '3本目の録画停止fallbackと自動summaryの元判定は保持。詳細はreview.json。次は右135度出口。\n',encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state=result['state'],same_candidate_valid_runs=3,same_candidate_capture_count=3,remaining_same_candidate_repeats=0,
    third_video_review_pending=False,same_candidate_three_review=str(OUT/'review.json'),next_motion='out_r135',
    next_action='右入口の単体相対候補3本を保存。右135度出口の今回配置・干渉なし・ON・手離しを確認後、固有IDで初回測定。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(angles_deg=angles.tolist(),mean_angle_deg=float(angles.mean()),maximum_Lend_residual_mm=result['Lend']['maximum_residual_mm']),ensure_ascii=False))
