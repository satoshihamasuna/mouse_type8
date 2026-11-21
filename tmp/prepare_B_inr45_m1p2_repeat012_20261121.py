"""補正−1.2度を固定した充電後の同条件反復012を準備。実行なし。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'geometry05_correction_m1p2';OUT=BASE/'geometry05_repeat01_correction_m1p2';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
review_path=BASE/'correction_m1p2_single_review_v1/review.json';review=read(review_path);current=review['rows'][-1]
assert current['run_id']=='011' and current['geometry']['independent_optical_geometry_eligible']
assert current['operator']['both_powered_off'] and current['operator']['no_contact']
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'))
assert all(candidate[g]==current['settings'][g] for g in ('parameters','pid','feedforward','suction','turn_count'))
assert candidate['parameters']['correction']==-1.2
candidate['source']='B1600 inR45：011と同じ補正−1.2度・Ls7.8/Le34.5mm・制御/吸引を固定。充電後の反復で出口方向のばらつきを調べる未実行012。'
save(OUT/'candidate.settings.json',candidate);config=read(PREVIOUS/'config.json');save(OUT/'config.json',config)
identifier='B_INR45_1600_m1p2_repeat01_green_20261121_012'
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_correction_m1p2_green_20261121_011',identifier)
compile(source,str(OUT/'run_recorded_once.py'),'exec');(OUT/'run_recorded_once.py').write_text(source,encoding='utf-8')
shutil.copyfile(PREVIOUS/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,current_placement_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0,
 comparison=dict(phase='same_candidate_after_charge_reproducibility',changed_fields=[],source_trial=current['source_trial'],
 previous=current['settings']['parameters'],requested=candidate['parameters']),new_one_run_readiness_required=True,
 charging_completion_required=True,charging_completion_confirmed=False)
save(OUT/'placement_plan.json',plan)
mode_paths=[BASE/k/'local_execution/runtime_modes.json' for k in ('geometry03_correction_m0p4','geometry04_correction_m0p9','geometry05_correction_m1p2')]
modes=[read(p) for p in mode_paths];assert all(m==modes[0] for m in modes)
diagnostic_paths=[BASE/'exit_window_diagnostic_v1'/(k+'.json') for k in ('geometry03_correction_m0p4','geometry04_correction_m0p9','geometry05_correction_m1p2')]
diagnostics=[read(p) for p in diagnostic_paths]
basis=dict(state='PREPARED_NOT_EXECUTED',source_report=str(review_path),parameters_changed=False,
 motivation='補正−0.4/−0.9度は出口44.208/44.233度、−1.2度は46.527度。追加の補正値変更を止め、同じ−1.2度の反復で再現性を調べる。',
 same_candidate_first_run=current['source_trial'],same_candidate_first_exit_deg=current['geometry']['exit_path_direction_change_deg'],
 same_candidate_first_Lend_mm=current['geometry']['midpoint_Lend_right_forward_mm'],
 previous_battery_min_v=current['tracking']['battery_min_v'],charging_requested=True,charging_completion_confirmed=False,
 runtime_modes_identical_for_009_010_011=True,runtime_modes=modes[0],
 exit_window_diagnostics=[dict(source_trial=d['source_trial'],windows=d['windows'],acceptance_changed=False) for d in diagnostics],
 short_windows_not_used_for_acceptance=True,original_thresholds_and_window_unchanged=True,source_params_changed=False,hardware_commands_sent=0,
 input_sha256={str(p):sha(p) for p in [review_path,Path(__file__)]+mode_paths+diagnostic_paths})
save(OUT/'repetition_basis.json',basis)
save(OUT/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',request_id=identifier,request_sent=False,
 differences={},same_settings_as_011=True,charging_completion_required=True,hardware_commands_sent=0,source_params_changed=False,firmware_written=False,
 input_sha256={str(p):sha(p) for p in (OUT/'config.json',OUT/'candidate.settings.json',OUT/'run_recorded_once.py',Path(config['geometry']),OUT/'repetition_basis.json',review_path)}))
save(BASE/'correction_m1p2_single_review_v1/next_decision.json',dict(decided_at_utc=datetime.now(timezone.utc).isoformat(),
 current_correction_adopted=False,next_stage=str(OUT),next_request_id=identifier,request_sent=False,settings_changed=False,
 charging_required=True,reason=basis['motivation']))
(OUT/'README.md').write_text('# 機体B1600 右in45：補正固定の反復012準備\n\n'
 '011の補正−1.2度・Lstart7.8/Lend34.5mm・半径−55mm・速度1.6m/s・PID/FF・吸引650をそのまま使う。追加の角度/長さ変更は行わず、充電後に一回反復する。\n\n'
 '011は相対出口46.527度、Lend46.467/178.411mm、必要区間の実観測/ArUco登録率99.31%。登録不成立1フレームは除外。最低電圧7.492Vのため両方OFFで充電を案内した。\n\n'
 '009/010/011の記録済みruntime_modesは同一。出口窓の感度も保存したが、短い窓は診断のみで主判定へ使わない。同じ95%基準・各フレーム実ArUco・入口raw wheel時計・出口窓・20ms補間上限を維持する。\n\n'
 '前緑/後赤、高さ1500mm、EV−1.5/1.8倍、B用E1:7F:86:21:84:73 / BLE8767、カメラ8765。充電完了後、両方OFFで元の中点225/45mm・前緑下向きへ概略配置し、今回の干渉余裕・両方ON・手離し・一回可を確認して012を一回取得する。011は再送しない。\n\n'
 '反復・左・連続接続・BのEKFは未確認。Params/ファーム変更なし。元の標準summary/auto_stop判定を保持し、取得時停止証拠・後半静止像・操作者OFF/接触なしと区別して保存。\n',encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);save(OUT/'resume_state.before_preparation.json',state)
state.update(state='M1P2_FIXED_REPETITION_012_PREPARED_AWAIT_CHARGING_AND_READY',next_stage=str(OUT),next_request_id=identifier,next_request_sent=False,
 next_operator_ready=False,operator_ready=False,next_settings=str(OUT/'candidate.settings.json'),latest_aggregate_review=str(review_path),
 latest_review=str(Path(current['source_trial']).parent/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),next_geometry_basis=str(OUT/'repetition_basis.json'),
 latest_standard_full_summary=current['full_summary'],archive=read(Path(current['source_trial']).parent/'archive.json'),
 physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,charging_requested=True,charging_completion_confirmed=False,
 charging_request_source_trial=current['source_trial'],latest_battery_min_v=current['tracking']['battery_min_v'],
 current_correction_same_condition_runs=1,remaining_current_correction_repeats=2,
 next_action='両方OFFで充電完了を確認し、同じ−1.2度/7.8/34.5mmの012を一回取得して出口角の再現性を調べる。新規の配置/ON/手離し/一回可を確認する。')
state['readiness_question_pending']=None
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n011を一回取得し両方OFF/接触なし。出口46.527度、Lend46.467/178.411mm。必要区間実観測/ArUco99.31%、不成立1フレームは除外。最低7.492Vで充電案内。角度/長さ変更を止め、充電後に同じ−1.2度を012で反復する準備。correction_m1p2_single_review_v1/README.md、geometry05_repeat01_correction_m1p2/README.md。\n')
print(json.dumps(dict(stage=str(OUT),same_settings_as_011=True,correction=-1.2,request_sent=False,charging_required=True),ensure_ascii=False))
