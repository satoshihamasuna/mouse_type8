"""実測出口角を挟む2条件から−0.9度の010を準備。実行はしない。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'geometry03_correction_m0p4';OUT=BASE/'geometry04_correction_m0p9';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
review_path=BASE/'correction_m0p4_single_review_v1/review.json';review=read(review_path)
new=review['after'];assert new['geometry']['independent_optical_geometry_eligible'] and new['operator']['both_powered_off'] and new['operator']['no_contact']
old_angles=np.array([row['geometry']['exit_path_direction_change_deg'] for row in review['before']]);new_angle=new['geometry']['exit_path_direction_change_deg']
c0=-1.5;c1=-.4;y0=float(old_angles.mean());y1=float(new_angle);slope=(y1-y0)/(c1-c0)
assert y1<45<old_angles.min() and slope<0
unrounded=c1+(45-y1)/slope;correction=round(unrounded,1);assert correction==-.9 and c0<correction<c1
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'));candidate['parameters']['correction']=correction
candidate['source']='B1600 inR45：補正−1.5度の出口平均46.072度と−0.4度の44.208度を挟む線形内挿候補−0.9度。長さ・制御・吸引は固定。未実行。'
save(OUT/'candidate.settings.json',candidate)
config=read(PREVIOUS/'config.json');assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
save(OUT/'config.json',config)
identifier='B_INR45_1600_correction_m0p9_green_20261121_010'
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_correction_m0p4_green_20261121_009',identifier)
compile(source,str(OUT/'run_recorded_once.py'),'exec');(OUT/'run_recorded_once.py').write_text(source,encoding='utf-8')
shutil.copyfile(PREVIOUS/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,current_placement_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0,
 comparison=dict(phase='bracketed_angle_correction_only',changed_fields=['parameters.correction'],previous=new['settings']['parameters'],
 requested=candidate['parameters'],basis=str(OUT/'angle_basis.json')),new_one_run_readiness_required=True)
save(OUT/'placement_plan.json',plan)
basis=dict(state='PREPARED_NOT_EXECUTED',mouse='B',motion='in_r45',preset=1600,
 old_correction_deg=c0,old_exit_deg=old_angles.tolist(),old_mean_exit_deg=y0,new_correction_deg=c1,new_exit_deg=y1,
 observed_exit_sensitivity_deg_per_correction_deg=float(slope),unrounded_secant_candidate_deg=float(unrounded),requested_correction_deg=correction,
 clean_captures_only_secant_candidate_deg=float(c1+(45-y1)/((y1-float(old_angles[:2].mean()))/(c1-c0))),
 per_old_run_secant_candidates_deg=[float(c1+(45-y1)/((y1-angle)/(c1-c0))) for angle in old_angles],
 target_exit_deg=45.,projected_exit_deg=float(y1+slope*(correction-c1)),Lstart_mm=7.8,Lend_mm=34.5,
 source_report=str(review_path),source_report_sha256=sha(review_path),
 assumptions=['同じ長さ/制御条件の小角度区間を線形近似。−0.4条件は単発で感度は未検証。',
 '旧−1.5度の008は連続必要区間の独立評価のみ。正常録画006/007だけでも丸め候補は−0.9度。',
 '出口角・Lend・観測品質は次の一回で再確認。長さの追加変更は角度確認後へ分ける。'],
 source_params_changed=False,hardware_commands_sent=0)
save(OUT/'angle_basis.json',basis)
diff={group:{key:[new['settings'][group][key],value] for key,value in candidate[group].items() if new['settings'][group][key]!=value}
 for group in ('parameters','pid','feedforward','suction')}
assert {g:d for g,d in diff.items() if d}=={'parameters':{'correction':[-.4,-.9]}}
save(OUT/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,differences=diff,
 hardware_commands_sent=0,source_params_changed=False,firmware_written=False,input_sha256={str(p):sha(p) for p in (OUT/'config.json',
 OUT/'candidate.settings.json',OUT/'run_recorded_once.py',Path(config['geometry']),OUT/'angle_basis.json',review_path,Path(__file__))}))
decision=dict(decided_at_utc=datetime.now(timezone.utc).isoformat(),source_trial=new['source_trial'],single_new_correction_adopted=False,
 reason='−0.4度は出口44.208度で目標を下回った。旧−1.5度条件との間を内挿し、角度だけ−0.9度で再確認する。',
 next_stage=str(OUT),next_request_id=identifier,request_sent=False)
save(BASE/'correction_m0p4_single_review_v1/next_decision.json',decision)
(OUT/'README.md').write_text('# 機体B1600 右in45：010準備\n\n'
 f'角度補正だけ−0.4→−0.9度。旧−1.5度の出口平均{y0:.3f}度と、009の{y1:.3f}度から線形内挿（未丸め{unrounded:.3f}度、予測出口{basis["projected_exit_deg"]:.3f}度）。実走未確認の候補。\n\n'
 'Lstart7.8/Lend34.5mm、半径−55mm、速度1.6m/s、PID/FF、吸引650を固定。Bの前緑/後赤・検証済み赤H160色域を使用。95%観測率・各フレーム実ArUco登録・20ms補間上限・入口raw wheel時計は維持。\n\n'
 'BLEはB用E1:7F:86:21:84:73 / 8767、カメラ8765。使用中のBポートへは接続しない。009は実行済みで再送しない。現在は両方OFF・接触なし確認済み。元225/45mm・前緑下向きへ概略配置し、新規の両方ON・手離し・一回可の返答後に010を一回取得する。\n\n'
 '新角度の反復・左・連続接続・BのEKFは未確認。Params/ファームは変更していない。\n',encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);save(OUT/'resume_state.before_preparation.json',state)
state.update(state='010_CORRECTION_M0P9_PREPARED_NOT_EXECUTED',operator_ready=False,next_operator_ready=False,
 next_stage=str(OUT),next_request_id=identifier,next_request_sent=False,next_settings=str(OUT/'candidate.settings.json'),
 latest_review=str(Path(new['source_trial']).parent/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),
 latest_aggregate_review=str(review_path),latest_single_new_correction_runs=1,
 next_geometry_basis=str(OUT/'angle_basis.json'),archive=read(Path(new['source_trial']).parent/'archive.json'),
 next_action='009の出口44.208度を確認。長さ固定で補正−0.9度の010を準備し、現状の概略配置・両方ON・手離し・一回可を確認後に取得する。',
 physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False)
state['readiness_question_pending']=None
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n009を一回取得し両方OFF/接触なし。録画正常、必要区間95.17%/ArUco100%。補正−0.4度の出口44.208度、Lend43.077/178.573mm。旧−1.5度条件との内挿で次は−0.9度のみの未実行010を準備。correction_m0p4_single_review_v1/README.md、geometry04_correction_m0p9/angle_basis.json。\n')
print(json.dumps(dict(stage=str(OUT),correction=correction,unrounded_candidate=float(unrounded),predicted_exit_deg=basis['projected_exit_deg'],request_sent=False),ensure_ascii=False))
