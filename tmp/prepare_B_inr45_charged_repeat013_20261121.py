"""S30検証済みプロファイルで同設定013を準備。明示的な充電完了前の実行を止める。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'geometry05_repeat01_correction_m1p2';OUT=BASE/'geometry05_repeat02_correction_m1p2';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
review_path=BASE/'red_s30_validation_v1/review.json';review=read(review_path);latest=review['rows'][-1]
assert latest['run_id']=='012' and latest['geometry']['independent_optical_geometry_eligible']
assert not latest['charging_condition']['charging_completed_before_trial'] and latest['operator']['both_powered_off']
identifier='B_INR45_1600_m1p2_repeat02_green_20261121_013'
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'))
assert all(candidate[g]==latest['settings'][g] for g in ('parameters','pid','feedforward','suction','turn_count'))
candidate['source']='B1600 inR45：補正−1.2度・Ls7.8/Le34.5mm・制御/吸引を固定。充電完了を明示確認してから013を一回取得。撮影/解析は検証済み赤S30プロファイルを使用。'
save(OUT/'candidate.settings.json',candidate)
config=read(PREVIOUS/'config.json');config['geometry']=str(BASE/'body_geometry_B_green_red_h160_s30_validated_v5.json')
assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
assert read(config['geometry'])['analysis_profile_adoption_state']=='VALIDATED_OFFLINE_FOR_THIS_B_CAPTURE_CONDITION'
save(OUT/'config.json',config)
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_m1p2_repeat01_green_20261121_012',identifier)
needle='args = parser.parse_args()\n';assert source.count(needle)==1
guard="""args = parser.parse_args()
if args.operator_ready:
    charge_path = OUT/'charging_completion_confirmed.json'
    if not charge_path.is_file():
        raise RuntimeError('機体Bの充電完了の明示確認が未記録です。接続・録画・設定・走行は開始しません')
    charge = tuning.read_json(charge_path)
    if charge.get('charging_completed') is not True or charge.get('explicit_user_confirmation') is not True or charge.get('request_id') != args.trial_id:
        raise RuntimeError('今回の要求IDに対応する充電完了の明示確認が必要です')
"""
source=source.replace(needle,guard);compile(source,str(OUT/'run_recorded_once.py'),'exec');(OUT/'run_recorded_once.py').write_text(source,encoding='utf-8')
shutil.copyfile(PREVIOUS/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,current_placement_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0,
 comparison=dict(phase='same_control_settings_after_explicit_charging_completion',changed_fields=[],source_trial=latest['source_trial'],
 previous=latest['settings']['parameters'],requested=candidate['parameters']),geometry_profile=config['geometry'],
 charging_completion_required=True,charging_completion_confirmed=False,new_one_run_readiness_required=True)
save(OUT/'placement_plan.json',plan)
save(OUT/'repetition_basis.json',dict(state='PREPARED_NOT_EXECUTED_AWAIT_EXPLICIT_CHARGING_COMPLETION',
 source_report=str(review_path),source_report_sha256=sha(review_path),same_control_settings_as_011_012=True,
 analysis_profile_changed_from_v4_to_validated_v5=True,only_analysis_red_saturation_changed=[40,30],
 original012_v4_rejection_preserved=True,original012_uncharged_condition_preserved=True,
 current_s30_same_settings_summary=review['current_condition'],charging_completion_confirmed=False,hardware_commands_sent=0))
save(OUT/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,differences={},
 same_control_settings_as_012=True,analysis_profile=str(config['geometry']),charging_completion_required=True,
 explicit_charging_confirmation_gate_present=True,charging_confirmation_file_exists=False,hardware_commands_sent=0,source_params_changed=False,firmware_written=False,
 input_sha256={str(p):sha(p) for p in (OUT/'config.json',OUT/'candidate.settings.json',OUT/'run_recorded_once.py',Path(config['geometry']),
 OUT/'repetition_basis.json',review_path,Path(__file__))}))
(OUT/'README.md').write_text('# 機体B1600 右in45：充電完了確認後の013準備\n\n'
 '制御設定は補正−1.2度・Lstart7.8/Lend34.5mm・半径−55mm・速度1.6m/s・PID/FF・吸引650を固定。撮影/解析用のB赤マーカー彩度だけ、実画像で検証済みのS30プロファイルへ切り替える。元の012/旧S40の不成立は変更しない。\n\n'
 '別解析で必要区間実観測率は010/011/012で100/99.31/100%。実フレーム監査と過去独立2本の出口/位置差を確認済み。S30で同条件−1.2度の出口は46.354/45.666度、Lend46.077/178.372・45.928/178.321mm。012は充電未完了との明示回答あり。\n\n'
 '現在は両方OFFで充電完了待ち。充電完了の明示回答をこの013用charging_completion_confirmed.jsonへ記録するまで、--operator-readyでも接続・録画・設定・走行を開始しない。通常起動はオフライン検証のみ。\n\n'
 '充電完了後にカメラ健康状態/IDLE・Bポート空きを再確認し、元225/45mm・前緑下向きへ概略配置、干渉余裕、両方ON・手離し・一回可の現状を確認して一回取得する。BLEはB用E1:7F:86:21:84:73 / 8767、カメラ8765。012は再送しない。\n\n'
 '充電後の再現性、幾何採用値、左・連続接続、BのEKFは未確認。Params/ファーム変更なし。95%・実ArUco・20ms補間上限・入口raw wheel時計・出口窓は維持。\n',encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);save(OUT/'resume_state.before_preparation.json',state)
state.update(state='013_PREPARED_AWAIT_EXPLICIT_CHARGING_COMPLETION_BOTH_OFF',next_stage=str(OUT),next_request_id=identifier,
 next_request_sent=False,next_settings=str(OUT/'candidate.settings.json'),next_operator_ready=False,operator_ready=False,
 next_geometry_basis=str(OUT/'repetition_basis.json'),latest_aggregate_review=str(review_path),latest_preparation=str(OUT/'offline_validation.json'),
 next_camera_health_report=None,next_camera_health_check_due=True,charging_completion_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,current_correction_same_condition_runs=2,
 next_action='両方OFFで充電完了の明示回答待ち。完了後に013用確認記録・カメラ健康状態/IDLE・Bポート空きを照合し、現在配置と手離しを確認して同設定を一回取得する。',
 readiness_question_pending=dict(request_id=identifier,phase='CHARGING_COMPLETION_ONLY_KEEP_BOTH_OFF',answered=False,
  source='両方OFFのまま、機体Bの充電が完了したら教えてください。現時点でONを依頼しない。'))
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n012は充電未完了との明示回答。旧S40の観測90.28%による不成立を保持。実赤S30画素に基づく別解析を010/011で検証し、012の必要区間100%、出口45.666度・Lend45.928/178.321mmを得た。S30結果はred_s30_validation_v1/README.md。013は同じ制御設定で、充電完了の明示確認がないと実行しない準備。現在両方OFF。\n')
print(json.dumps(dict(stage=str(OUT),request_sent=False,same_control_settings=True,explicit_charging_confirmation_required=True),ensure_ascii=False))
