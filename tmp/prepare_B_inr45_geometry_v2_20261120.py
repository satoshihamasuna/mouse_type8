"""入口相対動画と小角度の幾何感度からB1600 inR45の未実行形状候補を作る。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'ff750_omki0p005_01'
RUN=PREVIOUS/'local_execution/run_B_INR45_1600_ff750_ki005_green_20261120_004'
OUT=BASE/'geometry02'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
actual=read(RUN/'saved_1ms.settings.json')
geometry=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
assert geometry['independent_optical_geometry_eligible']
assert read(RUN/'operator_after_run.json')['both_powered_off']
p=actual['parameters'];theta_old=np.radians(geometry['exit_path_direction_change_deg'])
correction_new=round(p['correction']+geometry['exit_path_direction_change_deg']-45.,1)
old_command_deg=abs(p['degree']+p['correction'])
new_command_deg=abs(p['degree']+correction_new)
command_delta_deg=new_command_deg-old_command_deg
theta_new=theta_old+np.radians(command_delta_deg)
raw=pd.read_csv(RUN/'saved_1ms.csv')
omega=np.abs(raw['ideal.rad_velo'].to_numpy())
tracking=read(RUN/'tracking_1ms_review_v3/verification.json')['tracking']
curve=(raw.cnt.to_numpy()>=int(tracking['start']))&(raw.cnt.to_numpy()<=int(tracking['end']))
assert (raw.loc[curve,'motion.pattern']==7).all()
omega=omega[curve]
assert len(omega)>20
phase=(np.cumsum(omega)-.5*omega)/omega.sum()
tick=geometry['clock']['measured_tick_ms']/1000
velo=float(raw.loc[curve,'ideal.velo'].mean())
assert 30.<1000*velo*tick*len(omega)<100., 'Angular-profile path length sanity check failed'
ratio=new_command_deg/old_command_deg
old_phase=phase*np.radians(old_command_deg)
new_phase=phase*np.radians(new_command_deg)
old_arc=1000*velo*tick*np.array([np.sin(old_phase).sum(),np.cos(old_phase).sum()])
new_arc=1000*velo*tick*ratio*np.array([np.sin(new_phase).sum(),np.cos(new_phase).sum()])
direction=lambda angle:np.array([np.sin(angle),np.cos(angle)])
arc_change=new_arc-old_arc
exit_length_change=p['lend']*(direction(theta_new)-direction(theta_old))
predicted_before_lengths=np.array(geometry['midpoint_Lend_right_forward_mm'])+arc_change+exit_length_change
target=np.array([45.,180.])
delta_le=float((target[0]-predicted_before_lengths[0])/np.sin(theta_new))
delta_ls=float(target[1]-predicted_before_lengths[1]-delta_le*np.cos(theta_new))
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'))
for key in ('parameters','pid','feedforward','suction','turn_count'):
    candidate[key]=copy.deepcopy(actual[key])
candidate['parameters'].update(lstart=round(p['lstart']+delta_ls,1),lend=round(p['lend']+delta_le,1),correction=correction_new)
assert candidate['parameters']['lstart']>0 and candidate['parameters']['lend']>0
candidate['source']='B1600 inR45 Ki0.005の入口相対動画を基準にした未実行形状候補。小角度変更に対する理想旋回の幾何感度を加え、前後長を同時に解く。効果は次の一回走行で確認する。'
save(OUT/'candidate.settings.json',candidate)
for name in ('config.json','coarse_placement_check.py'):
    shutil.copyfile(PREVIOUS/name,OUT/name)
identifier='B_INR45_1600_geometry02_green_20261120_006'
script=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_ff750_ki005_green_20261120_004',identifier)
(OUT/'run_recorded_once.py').write_text(script,encoding='utf-8')
plan=read(PREVIOUS/'placement_plan.json')
plan.update(request_id=identifier,operator_ready=False,current_placement_confirmed=False,
    execution_commands_sent=0,settings_write_commands_sent=0,
    comparison=dict(phase='geometry_with_fixed_control',changed_fields=['parameters.lstart','parameters.lend','parameters.correction'],
        previous=actual['parameters'],requested=candidate['parameters'],previous_trial=str(RUN/'trial.json')))
save(OUT/'placement_plan.json',plan)
predicted_rounded=predicted_before_lengths+np.array([0.,candidate['parameters']['lstart']-p['lstart']])+(candidate['parameters']['lend']-p['lend'])*direction(theta_new)
save(OUT/'geometry_basis.json',dict(state='PREPARED_NOT_EXECUTED',source_trial=str(RUN/'trial.json'),
    observed_exit_deg=geometry['exit_path_direction_change_deg'],observed_Lend_right_forward_mm=geometry['midpoint_Lend_right_forward_mm'],
    target_right_forward_mm=target.tolist(),target_includes_pre_run_90mm=True,
    requested_correction_deg=correction_new,command_angle_change_deg=command_delta_deg,
    projected_exit_angle_deg=float(np.degrees(theta_new)),
    curve_selection=dict(first_cnt=int(tracking['start']),last_cnt=int(tracking['end']),sample_count=int(curve.sum()),motion_pattern=7,pre_and_post_heading_correction_excluded=True),
    ideal_arc_change_right_forward_mm=arc_change.tolist(),existing_exit_length_projection_change_mm=exit_length_change.tolist(),
    delta_Lstart_mm=delta_ls,delta_Lend_mm=delta_le,rounded_parameters=candidate['parameters'],
    projected_rounded_Lend_right_forward_mm=predicted_rounded.tolist(),
    measured_clock=geometry['clock'],timing_sensitivity=geometry['timing_sensitivity'],
    assumptions=['指令角の小変更に対する光学出口角の応答を1対1と近似する。未検証。',
        '旋回前後の速度・半径・閉ループ追従の条件が保たれ、現試行の系統誤差が小変更の前後で一定と近似。',
        '理想角速度の正規化波形を使い、指令角に比例する旋回時間の変化を幾何感度に含める。',
        '動画中点の入口相対候補。絶対車軸位置・body yaw・独立同期の保証ではない。'],
    input_sha256={str(q):hashlib.sha256(q.read_bytes()).hexdigest() for q in (
        RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json',Path(__file__))}))
diff={group:{k:[actual[group][k],v] for k,v in candidate[group].items() if actual[group][k]!=v} for group in ('parameters','pid','feedforward','suction')}
diff={k:v for k,v in diff.items() if v}
assert set(diff)=={'parameters'} and set(diff['parameters'])=={'lstart','lend','correction'}
save(OUT/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',differences=diff,
    preserved=['radius','velocity','logical_degree','PID','feedforward','suction','count'],hardware_commands_sent=0,operator_ready=False))
state=read(BASE/'resume_state.json')
state.update(state='KI005_CONTROL_CANDIDATE_GEOMETRY_PENDING',physical_power_state='BOTH_OFF_USER_CONFIRMED',
    execution_pending=False,execution_commands_sent=3,next_stage=str(OUT),next_settings=str(OUT/'candidate.settings.json'),
    next_request_id=identifier,next_request_sent=False,next_operator_ready=False,
    latest_review=str(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),
    next_action='元225/45mm・前緑下向きへ概略配置。干渉なし・両方ON・手離しの新規確認後、一回だけ形状候補を取得する。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
review_dir=BASE/'ki005_comparison_v2';review_dir.mkdir(exist_ok=False)
rows=[]
for sub in ('baseline02_relative/local_execution/run_B_INR45_1600_relative_green_20261120_002',
    'ff_om_accdec0p00075_01/local_execution/run_B_INR45_1600_ffaccdec750_green_20261120_003',
    'ff750_omki0p005_01/local_execution/run_B_INR45_1600_ff750_ki005_green_20261120_004'):
    folder=BASE/sub;track=read(folder/'tracking_1ms_review_v3/verification.json');geom=read(folder/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
    rows.append(dict(source_trial=str(folder/'trial.json'),settings=read(folder/'saved_1ms.settings.json'),tracking=track,
        geometry=geom,operator=read(folder/'operator_after_run.json')))
save(review_dir/'review.json',dict(rows=rows,next_stage=str(OUT),next_request_id=identifier,
    state='CONTROL_CANDIDATE_SELECTED_GEOMETRY_NOT_EXECUTED',selected_control=dict(pid=candidate['pid'],feedforward=candidate['feedforward']),
    source_params_changed=False,firmware_written=False,independent_repeats_pending=True))
text='# 機体B turn1600 inR45：Iゲイン比較と形状候補\n\n'
text+='角加減速FF0.000750を保ち、角速度Kiを0.010→0.005へ変更。旋回RMS5.596→4.606 rad/s、ピーク37.531→34.906 rad/s、後揺れ負ピーク-12.727→-9.883 rad/s。'
text+='動画の入口相対出口角は44.411度、Lendは右47.892/前181.489 mm（目標45/180）。各条件は単発で、Ki0.005を形状調整用候補とする。\n\n'
text+=f"次の未実行候補はLstart {candidate['parameters']['lstart']:.1f} mm、Lend {candidate['parameters']['lend']:.1f} mm、角度補正 {correction_new:+.1f}度。"
text+='角度変更の小さい幾何感度を理想波形から加え、実測残差に対して前後長を解いた候補。geometry02/geometry_basis.jsonに近似条件・時刻感度・入力ハッシュを保存。'
text+='追従候補の制御値は固定し、次の一回で角度と長さを併せて検証する。独立反復・連続接続・BのEKF同定・ファーム反映は未実施。\n\n'
text+='各評価コマで実ArUco登録を必須とする。元summaryの初期直進観測不足を保持し、撮影時停止確認1.323秒・後半静止動画105コマのscatter0.261mm・操作者の両方OFF/接触なしを別に記録。\n'
(review_dir/'README.md').write_text(text,encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\nKi0.005比較004を一回取得し、両方OFF・接触なし確認。追従/後揺れがFF750/Ki0.01より減少。入口相対出口44.411度、Lend差右+2.892/前+1.489mm。次の未実行形状候補と根拠はki005_comparison_v2/README.md、geometry02/geometry_basis.json。ファーム/Params変更なし。\n')
print(json.dumps(dict(stage=str(OUT),parameters=candidate['parameters'],projected_rounded_Lend=predicted_rounded.tolist()),ensure_ascii=False))
