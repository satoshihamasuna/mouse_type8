"""基準001から角度補正だけを変更した未送信002を準備する。機器接続なし。"""
from pathlib import Path
from datetime import datetime,timezone
import copy,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
PREVIOUS=BASE/'baseline01';STAGE=BASE/'correction02_m0p9';STAGE.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):
 with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
state_path=BASE/'resume_state.json';state=read(state_path)
assert state['execution_commands_sent']==1 and state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED'
RUN=Path(state['source_trial']).parent;review_path=BASE/'baseline01_review_v1/review.json';review=read(review_path)
assert review['visual_review_completed'] and review['angle_deg']<45
archive=read(RUN/'archive.json')
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive,verify_sources
verify_sources(verify_archive(archive['archive_folder']),RUN/'trial.json')
old_identifier='B_OUTR45_1600_baseline_green_20261121_001';identifier='B_OUTR45_1600_correction_m0p9_green_20261121_002'
old=read(PREVIOUS/'candidate.settings.json');candidate=copy.deepcopy(old)
candidate['parameters']['correction']=-.9
candidate['source']='B1600 outR45基準001の相対出口43.809度に基づく角度のみの未実行比較。Lstart21.8/Lend16.5mm・PID/FF・吸引を固定。'
diff={g:{k:[old[g][k],v] for k,v in candidate[g].items() if old[g][k]!=v} for g in ('parameters','pid','feedforward','suction')}
assert {g:v for g,v in diff.items() if v}=={'parameters':{'correction':[.3,-.9]}}
save(STAGE/'candidate.settings.json',candidate);config=read(PREVIOUS/'config.json');save(STAGE/'config.json',config)
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8-sig');assert old_identifier in source
source=source.replace(old_identifier,identifier);compile(source,str(STAGE/'run_recorded_once.py'),'exec')
with (STAGE/'run_recorded_once.py').open('x',encoding='utf-8') as f:f.write(source)
with (STAGE/'coarse_placement_check.py').open('xb') as f:f.write((PREVIOUS/'coarse_placement_check.py').read_bytes())
charge=read(PREVIOUS/'charging_completion_confirmed.json')
charge.update(request_id=identifier,original_request_id=old_identifier,original_confirmation_file=str(PREVIOUS/'charging_completion_confirmed.json'),
 source='同じ調整セッションで基準001前の明示充電完了確認を継続。現在まで実走一回。追加充電したという新しい確認ではない。')
save(STAGE/'charging_completion_confirmed.json',charge)
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',current_placement_confirmed=False,
 comparison=dict(changed_fields=['parameters.correction'],previous=.3,requested=-.9,baseline_review=str(review_path)),
 full_footprint_and_hole_clearance_verified=False,new_one_run_readiness_required=True)
save(STAGE/'placement_plan.json',plan)
save(STAGE/'angle_basis.json',dict(prepared_at_utc=datetime.now(timezone.utc).isoformat(),request_id=identifier,
 baseline_review=str(review_path),baseline_review_sha256=sha(review_path),
 measured_relative_exit_deg=review['angle_deg'],target_relative_exit_deg=45.,
 requested_correction_deg=-.9,response_slope_assumed=-1.,response_slope_verified=False,
 length_changes_applied=False,length_change_reason='現状のLend差は1.165mm。時刻1msだけでLend推定約1.8mmの差になり、角度変更でも位置が変わるため据え置く。',
 actual_confirmation_required=True,request_sent=False,robot_commands_sent=0,source_params_changed=False))
paths=[STAGE/name for name in ('candidate.settings.json','config.json','run_recorded_once.py','coarse_placement_check.py',
 'placement_plan.json','charging_completion_confirmed.json','angle_basis.json')]
paths += [Path(config['geometry']),Path(config['reference']),BASE/'analysis_policy.json',review_path,Path(__file__)]
save(STAGE/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,
 differences=diff,helper_syntax_passed=True,hardware_commands_sent=0,input_sha256={str(p):sha(p) for p in paths}))
text=(ROOT/'tmp/check_B_inr45_next_preflight_20261121.py').read_text(encoding='utf-8-sig')
text=text.replace('turn1600_B_inr45_20261120_v1',BASE.name).replace("'in_r45'","'out_r45'")
text=text.replace('前緑下向き','前緑右下45度').replace('同じ概略位置','概略90/45mm')
compile(text,'check_B_out45_next_preflight_20261121.py','exec')
with (ROOT/'tmp/check_B_out45_next_preflight_20261121.py').open('x',encoding='utf-8') as f:f.write(text)
save(STAGE/'resume_state.before_preparation.json',state)
state.update(state='OUT45_002_CORRECTION_M0P9_PREPARED_NOT_EXECUTED',next_stage=str(STAGE),next_request_id=identifier,
 next_request_sent=False,next_operator_ready=False,operator_ready=False,current_placement_confirmed=False,
 next_settings=str(STAGE/'candidate.settings.json'),next_geometry_basis=str(STAGE/'angle_basis.json'),
 archive=archive,readiness_question_pending=None,
 next_action='角度補正のみ−0.9度の比較002を未送信準備。カメラ電池/待機/ポート確認後、概略90/45・前緑右下・干渉余裕・両方ON手離し一回可を確認する。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(STAGE/'README.md').write_text('# B・1600 右out45：角度のみ比較002（未実行）\n\n角度補正+0.3→−0.9度のみ変更。Lstart21.8/Lend16.5mm、半径−60mm、角速度Ki0.01、角加減速FF0.000750、吸引650を固定。基準001の相対出口43.809度からの初回比例仮定であり、次の一回で確認する。\n\n開始中点右90/下45mm・前緑右下45度へ概略配置。95%実観測/各コマ実ArUco2個以上/固定10-10ms出口窓など基準001と同じ条件。B用 E1:7F:86:21:84:73 / 8767。新しい配置・ON・手離し確認まで未送信。基準001の許可を再利用しない。\n\n充電確認は基準001前の明示回答を同じセッションで継続（実走一回）。新しい追加充電の確認とはしない。Params/ファーム書込み変更なし。\n',encoding='utf-8')
print(json.dumps(dict(stage=str(STAGE),differences=diff,request_sent=False,baseline_executions=1,archive_verified=True),ensure_ascii=False))
