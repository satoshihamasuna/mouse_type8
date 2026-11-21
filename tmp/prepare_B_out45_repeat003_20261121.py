"""−0.9度の同一設定確認一回を準備。002の時刻不合格は変更しない。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
PREVIOUS=BASE/'correction02_m0p9';STAGE=BASE/'correction02_repeat01_m0p9';STAGE.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):
 with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
state_path=BASE/'resume_state.json';state=read(state_path)
assert state['execution_commands_sent']==2 and state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED'
RUN=Path(state['source_trial']).parent;review_path=BASE/'correction02_review_v1/review.json';review=read(review_path)
assert review['next_decision']['action']=='repeat_same_candidate_once' and not review['second_run_geometry_adopted']
assert not review['rows'][1]['geometry_eligible'] and review['visual_review_completed']
archive=read(RUN/'archive.json');sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive,verify_sources
verify_sources(verify_archive(archive['archive_folder']),RUN/'trial.json')
old_identifier='B_OUTR45_1600_correction_m0p9_green_20261121_002'
identifier='B_OUTR45_1600_m0p9_repeat01_green_20261121_003'
candidate=read(PREVIOUS/'candidate.settings.json')
assert candidate['parameters']['correction']==-.9
candidate['source']='B1600 outR45 補正−0.9度の同一設定確認一回。002の時刻半窓差2.134ms>2msは不合格を保持。'
save(STAGE/'candidate.settings.json',candidate);config=read(PREVIOUS/'config.json');save(STAGE/'config.json',config)
for name in ('parameters','pid','feedforward','suction','turn_count'):
 assert candidate[name]==read(RUN/'saved_1ms.settings.json')[name],name
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8-sig');assert old_identifier in source
source=source.replace(old_identifier,identifier);compile(source,str(STAGE/'run_recorded_once.py'),'exec')
with (STAGE/'run_recorded_once.py').open('x',encoding='utf-8') as f:f.write(source)
with (STAGE/'coarse_placement_check.py').open('xb') as f:f.write((PREVIOUS/'coarse_placement_check.py').read_bytes())
charge=read(PREVIOUS/'charging_completion_confirmed.json')
charge.update(request_id=identifier,continued_after_request_id=old_identifier,
 source='基準001前の明示充電完了回答を同じセッションで継続。実走合計二回。追加充電を確認したという意味ではない。')
save(STAGE/'charging_completion_confirmed.json',charge)
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',current_placement_confirmed=False,
 comparison=dict(changed_fields=[],repeat_of_request_id=old_identifier,reference_review=str(review_path)),
 full_footprint_and_hole_clearance_verified=False,new_one_run_readiness_required=True)
save(STAGE/'placement_plan.json',plan)
save(STAGE/'repeat_basis.json',dict(prepared_at_utc=datetime.now(timezone.utc).isoformat(),request_id=identifier,
 source_review=str(review_path),source_review_sha256=sha(review_path),second_run_relative_geometry_eligible=False,
 reason='002は参考出口45.235度、時刻照合半窓差2.134msが上限2msを外れた。同じ設定一回で確認。',
 changed_fields=[],correction_deg=-.9,Lstart_mm=21.8,Lend_mm=16.5,
 quality_policy=read(BASE/'analysis_policy.json'),quality_thresholds_unchanged=True,request_sent=False,robot_commands_sent=0))
paths=[STAGE/name for name in ('candidate.settings.json','config.json','run_recorded_once.py','coarse_placement_check.py',
 'placement_plan.json','charging_completion_confirmed.json','repeat_basis.json')]
paths += [Path(config['geometry']),Path(config['reference']),BASE/'analysis_policy.json',review_path,Path(__file__)]
save(STAGE/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,
 differences={},same_parameters_as_previous_run=True,hardware_commands_sent=0,input_sha256={str(p):sha(p) for p in paths}))
save(STAGE/'resume_state.before_preparation.json',state)
state.update(state='OUT45_003_SAME_M0P9_CONFIRMATION_PREPARED_NOT_EXECUTED',next_stage=str(STAGE),next_request_id=identifier,
 next_request_sent=False,next_operator_ready=False,operator_ready=False,current_placement_confirmed=False,
 next_settings=str(STAGE/'candidate.settings.json'),next_geometry_basis=str(STAGE/'repeat_basis.json'),archive=archive,
 readiness_question_pending=None,
 next_action='003は−0.9度の同一設定確認一回。準備検証とカメラ状態確認後、今回の概略配置/干渉余裕/ON/手離しを確認して一回取得する。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(STAGE/'README.md').write_text('# B・1600 右out45：同条件確認003（未送信）\n\n002からパラメータ変更なし。補正−0.9度、Lstart21.8/Lend16.5mm、半径−60mm、角速度Ki0.01、角加減速FF0.000750、吸引650。002の参考出口45.235度に対し、時計半窓差2.134ms>2msを不合格のまま保持し、同条件一回を確認する。\n\n開始中点右90/下45mm・前緑右下45度へ概略配置。実観測95%以上/各評価コマ実ArUco2個以上/固定10-10ms出口窓/時刻品質を変更しない。BLE B E1:7F:86:21:84:73 / 8767。新しい配置・ON・手離し確認まで未送信。\n\n充電完了は基準001前の明示回答を同じセッションで継続（実走二回）。新しい追加充電確認ではない。Params/実機書込み変更なし。\n',encoding='utf-8')
print(json.dumps(dict(stage=str(STAGE),same_parameters=True,request_sent=False,executions_so_far=2,archive_verified=True),ensure_ascii=False))
