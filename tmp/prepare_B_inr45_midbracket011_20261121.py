"""実測で45度を挟む−1.5/−0.9度の中点−1.2度を、一回検証用に準備。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'geometry04_correction_m0p9';OUT=BASE/'geometry05_correction_m1p2';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
review_path=BASE/'correction_m0p9_single_review_v1/review.json';review=read(review_path);current=review['rows'][-1]
assert current['run_id']=='010' and current['geometry']['independent_optical_geometry_eligible']
assert current['operator']['both_powered_off'] and current['operator']['no_contact']
assert all(row['geometry']['exit_path_direction_change_deg']>45 for row in review['rows'][:3])
assert current['geometry']['exit_path_direction_change_deg']<45 and current['settings']['parameters']['correction']==-.9
correction=round((-1.5-.9)/2,1);assert correction==-1.2
identifier='B_INR45_1600_correction_m1p2_green_20261121_011'
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'));candidate['parameters']['correction']=correction
candidate['source']='B1600 inR45：目標45度を挟む補正−1.5度（出口45.945〜46.167度）と−0.9度（44.233度）の中点−1.2度。角度のみの未実行候補。'
save(OUT/'candidate.settings.json',candidate);config=read(PREVIOUS/'config.json')
assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
save(OUT/'config.json',config)
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_correction_m0p9_green_20261121_010',identifier)
compile(source,str(OUT/'run_recorded_once.py'),'exec');(OUT/'run_recorded_once.py').write_text(source,encoding='utf-8')
shutil.copyfile(PREVIOUS/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,current_placement_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0,
 comparison=dict(phase='midpoint_of_observed_angle_bracket',changed_fields=['parameters.correction'],previous=current['settings']['parameters'],
 requested=candidate['parameters'],basis=str(OUT/'angle_basis.json')),new_one_run_readiness_required=True)
save(OUT/'placement_plan.json',plan)
basis=dict(state='PREPARED_NOT_EXECUTED',method='midpoint of correction values whose measured relative exit angles bracket45',
 target_exit_deg=45.,bracket_correction_deg=[-1.5,-.9],bracket_old_exit_deg=[row['geometry']['exit_path_direction_change_deg'] for row in review['rows'][:3]],
 bracket_new_exit_deg=current['geometry']['exit_path_direction_change_deg'],requested_correction_deg=correction,
 previous_m0p4_exit_deg=review['rows'][-2]['geometry']['exit_path_direction_change_deg'],
 motivation='−0.4→−0.9度の出口変化は約0.025度だった。全区間の比例応答は仮定せず、実測で挟まれた区間を二分する。',
 Lstart_mm=7.8,Lend_mm=34.5,source_report=str(review_path),source_report_sha256=sha(review_path),
 reference_capture_008_scope='独立した必要連続区間のみ。正常録画006/007だけでも45度を挟む。全体FAILEDは保持。',
 standard_full_review_warning=current['full_summary'],
 independent_relative_review=dict(eligible=current['geometry']['independent_optical_geometry_eligible'],entry_line=current['geometry']['entry_line'],
 exit_line=current['geometry']['exit_line'],clock=current['geometry']['clock'],quality=current['geometry']['independent_clock_required_motion_quality']),
 assumptions=['各補正条件は撮影/制御/長さを揃えた相対マーカー中点の測定。現在の−0.9度は単発で、局所応答の再現性は未確認。',
 '−1.2度で目標を達成する保証ではない。次の一回で角度・長さ・品質を測定する。'],hardware_commands_sent=0,source_params_changed=False)
save(OUT/'angle_basis.json',basis)
diff={g:{k:[current['settings'][g][k],v] for k,v in candidate[g].items() if current['settings'][g][k]!=v} for g in ('parameters','pid','feedforward','suction')}
assert {g:d for g,d in diff.items() if d}=={'parameters':{'correction':[-.9,-1.2]}}
save(OUT/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,differences=diff,
 hardware_commands_sent=0,source_params_changed=False,firmware_written=False,input_sha256={str(p):sha(p) for p in (OUT/'config.json',OUT/'candidate.settings.json',
 OUT/'run_recorded_once.py',Path(config['geometry']),OUT/'angle_basis.json',review_path,Path(__file__))}))
save(BASE/'correction_m0p9_single_review_v1/next_decision.json',dict(decided_at_utc=datetime.now(timezone.utc).isoformat(),single_new_correction_adopted=False,
 next_stage=str(OUT),next_request_id=identifier,request_sent=False,reason=basis['motivation'],full_review_warning=current['full_summary']))
(OUT/'README.md').write_text('# 機体B1600 右in45：011準備\n\n角度補正だけ−0.9→−1.2度。実測で目標45度を挟む−1.5度/−0.9度の中点。−0.4/−0.9度の出口が44.21/44.23度で近かったため、比例感度の確定値として扱わない。\n\n'
 'Lstart7.8/Lend34.5mm・半径−55mm・速度1.6m/s・PID/FF・吸引650は固定。前緑/後赤、高さ1500mm、EV−1.5/1.8倍、検証済み赤H160色域。95%基準・実ArUco登録・20ms補間上限・入口raw wheel時計を維持。\n\n'
 '010は全体録画PASSED。元標準summaryは初期直進観測不足で要確認/auto_stop=falseを保持。別の入口raw wheel時計は25点/75.38mm・RMS0.195mm、出口17点/84.87mm・RMS0.516mmで相対形状の品質を通過。取得時の停止1.27秒・後半静止105点scatter0.188mm・操作者の両方OFF/接触なしを別記録。\n\n'
 'BLE E1:7F:86:21:84:73 / 8767、カメラ8765。未送信011を新規の元位置・両方ON・手離し・一回可の返答後に一回のみ取得。010は実行済みで再送しない。Params/ファーム変更なし。反復・左・連続接続・BのEKFは未確認。\n',encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);save(OUT/'resume_state.before_preparation.json',state)
state.update(state='011_CORRECTION_M1P2_PREPARED_NOT_EXECUTED',next_stage=str(OUT),next_request_id=identifier,next_request_sent=False,
 next_operator_ready=False,operator_ready=False,next_settings=str(OUT/'candidate.settings.json'),latest_aggregate_review=str(review_path),
 latest_review=str(Path(current['source_trial']).parent/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),next_geometry_basis=str(OUT/'angle_basis.json'),
 latest_standard_full_summary=current['full_summary'],latest_independent_relative_geometry_eligible=True,
 archive=read(Path(current['source_trial']).parent/'archive.json'),physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,
 next_action='010の出口44.233度・Lend差−1.051/−0.108mmを確認。角度だけ−1.2度の011を未送信準備。新規の現在配置/ON/手離し/一回可を確認する。')
state['readiness_question_pending']=None
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n010を一回取得し両方OFF/接触なし。録画正常、必要区間95.17%/ArUco100%。出口44.233度、Lend43.949/179.892mm。標準summaryは初期直進観測不足を保持、入口raw wheel時計の相対形状評価は通過。目標を挟む補正−1.5/−0.9度の中点−1.2度を未実行011候補に準備。correction_m0p9_single_review_v1/README.md、geometry05_correction_m1p2/README.md。\n')
print(json.dumps(dict(stage=str(OUT),correction=correction,request_sent=False,standard_full_warning_preserved=True),ensure_ascii=False))
