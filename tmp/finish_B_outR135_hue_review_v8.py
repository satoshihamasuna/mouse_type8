from pathlib import Path
from datetime import datetime, timezone
import json, hashlib, copy, sys, importlib.util, shutil
import numpy as np
ROOT=Path.cwd()
BASE=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A'
V=BASE/'marker_hue_revision_v8'
sys.path[:0]=[str(BASE),str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from review_outR135_board_fixed import diagnose
from prepare_outR135_candidate_fast import validate_files
def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:
        json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

validation=BASE/'zoom_camera_validation_v8.json'
assert load(validation)['state']=='PASSED_INDEPENDENT_STATIC_VALIDATION'
metas=[BASE/'next_session_B_outR135_corr1p5_Lstart33p7_v1.json',BASE/'next_session_B_outR135_corr1p5_Lstart33p7_repeat02_v1.json']
labels=['run01','repeat02']; results=[]; settings=[]; ids=[]; fv_paths=[]
for meta_path,label in zip(metas,labels):
    meta=load(meta_path); session=Path(meta['session']); run=session/('run_'+meta['capture_request_id'])
    assert meta['hardware_power']=='BOTH_OFF_USER_CONFIRMED' and meta['service_closed']
    assert not (session/'service.lock').exists()
    power=session/'power_off_confirmation.json'; assert load(power)['operator_reported_no_hole_or_wall_contact']
    reanalysis=V/f'outR135_{label}_reanalysis'; folder=V/f'outR135_{label}_review'
    review=load(folder/'review.json'); dense=V/f'outR135_{label}_dense/result.json'
    assert review['status']=='VALID_SINGLE_TRIAL_REVIEW' and load(dense)['state']=='PASSED'
    assert review['summary']['auto_stop_confirmed'] and review['summary']['motion_observation_rate']>=.95
    paths=[folder/'review.json',dense,power,validation,reanalysis/'reanalysis_provenance.json']
    validity=dict(state='VALID_RELATIVE_SINGLE_TRIAL_REANALYZED_V8',full_trajectory_review_passed=True,dense_static_confirmation_passed=True,
        same_settings_valid_trials=2,repeatability_verified=False,parameter_adoption_allowed=False,hardware_power='BOTH_OFF_USER_CONFIRMED',
        operator_reported_no_hole_or_wall_contact=True,board_fixed_goal_verified=False,evaluation_frame=meta['evaluation_frame'],
        original_native_report_unchanged=True,original_native_report_returncode=load(session/'jobs'/f"{meta['capture_request_id']}.json")['result']['full_returncode'],
        recording_id=load(run/'trial.json')['recording_id'],unique_physical_trial=True,independent_detector_validation=str(validation),
        relative_endpoint_error_right_forward_mm=[review['endpoint_error_right_mm'],review['endpoint_error_forward_mm']],exit_travel_error_deg=review['exit_travel_error_deg'],
        input_sha256={str(p):sha(p) for p in paths})
    fv=folder/'final_validity.json';save(fv,validity);fv_paths.append(fv)
    save(folder/'board_fixed_diagnostic.json',diagnose(reanalysis,Path(meta['plan'])))
    meta.update(state='VALID_RELATIVE_TRIAL_V8_SUPPLEMENTED_POWER_OFF_CLOSED',latest_final_validity=str(fv),
        original_final_validity_preserved=meta.get('final_validity'),latest_analysis_geometry=str(BASE/'body_geometry_B_zoom2_v8.json'),
        same_settings_valid_trials=2)
    meta_path.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    results.append(review); settings.append(load(run/'motion.settings.json'));ids.append(validity['recording_id'])
assert len(set(ids))==2
for k in ['parameters','pid','feedforward','suction','preset_speed_mm_s','turn_count','mouse','motion_name']:
    assert settings[0][k]==settings[1][k],k
errors=np.array([[r['endpoint_error_right_mm'],r['endpoint_error_forward_mm']] for r in results])
save(V/'two_trial_summary.json',dict(state='TWO_VALID_RELATIVE_TRIALS_THIRD_REQUIRED',recording_ids=ids,physical_trials=2,
    endpoint_errors_right_forward_mm=errors.tolist(),mean_endpoint_error_right_forward_mm=errors.mean(axis=0).tolist(),
    range_endpoint_error_right_forward_mm=np.ptp(errors,axis=0).tolist(),exit_errors_deg=[r['exit_travel_error_deg'] for r in results],
    measurement_detector='v8 yellow hue25..65; all other detector thresholds/physical settings unchanged',quality_thresholds_relaxed=False,
    firmware_active_parameters_changed=False,parameter_adoption_allowed=False,relative_only=True,board_fixed_goal_verified=False,
    next_step='One additional same-settings unique physical trial',input_sha256={str(p):sha(p) for p in fv_paths}))
save(V/'static_protocol_interpretation.json',dict(meaning='grid_and_colour_thresholds_unchanged in each new dense protocol means the same frozen v8 detector was used for both static and prerun footage; hue range differs from original v6. Original v6 outputs and failures remain unchanged.',old_geometry=str(BASE/'body_geometry_B_zoom2_v6.json'),new_geometry=str(BASE/'body_geometry_B_zoom2_v8.json')))

# Keep a warning with the earlier candidate rather than silently reusing a known PCB false detection.
oldmeta=BASE/'B_inR135_single_turn_candidate.meta.json'
shutil.copy2(oldmeta,V/'B_inR135_candidate_meta_before_detector_warning.json')
old=load(oldmeta); old.update(measurement_detector_followup_required=True,
    detector_followup_reason='The old 2x inR135 clip contains PCB-yellow false front detections; preserve old result, remeasure with validated physical-marker detector before claiming continuous or fixed-board acceptance.',
    detector_followup_evidence=str(V/'independent_visual_marker_validation.json'))
oldmeta.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

previous=load(metas[1]); plan=load(previous['plan']); source_run=Path(previous['session'])/('run_'+previous['capture_request_id'])
session=ROOT/'tools/logs/pixel_camera/tuning_cli_B_outR135_corr1p5_Lstart33p7_repeat03_v8_20261115_prepared'
session.mkdir(exist_ok=False)
plan_path=BASE/'B_outR135_corr1p5_Lstart33p7_repeat03_v8_plan.json'
meta_path=BASE/'next_session_B_outR135_corr1p5_Lstart33p7_repeat03_v8.json'
helper=BASE/'preflight_outR135_corr1p5_Lstart33p7_repeat03_v8_zoom2.py'
config_path=BASE/'tuning_cli_config_B_outR135_zoom2_v8.json'; config=load(previous['config']);config['geometry']=str(BASE/'body_geometry_B_zoom2_v8.json');save(config_path,config)
plan.update(state='UNEXECUTED_SAME_SETTINGS_THIRD_TRIAL_VALIDATED_DETECTOR_V8',baseline_actual_settings=str(source_run/'motion.settings.json'),
    baseline_final_validity=str(fv_paths[1]),baseline_actual_settings_sha256=sha(source_run/'motion.settings.json'),baseline_final_validity_sha256=sha(fv_paths[1]),
    changes_from_last_trial={},comparison_reason='Two unique recordings reanalyzed with identical independently reviewed v8 detector; both within1mm and0.04deg. Third same-settings trial; no geometry retuning.',
    detector_validation=str(validation),same_settings_valid_trials_before_capture=2)
save(plan_path,plan)
new=copy.deepcopy(previous)
for k in ['recording_id','trial','final_validity','service_pid','service_closed','original_final_validity_preserved','latest_final_validity','latest_analysis_geometry','same_settings_valid_trials']:
    new.pop(k,None)
new.update(state='OFFLINE_PREPARED_AWAITING_FRESH_OPERATOR_READY',session=str(session),config=str(config_path),plan=str(plan_path),
    capture_request_id='B_outR135_corr1p5_Lstart33p7_repeat03_v8',operator_ready='PENDING_FRESH_CONFIRMATION',
    hardware_power='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,execution_resends=0,preflight_ready=False,
    resume_allowed=True,operator_hole_clearance_pending=True,preflight_helper=str(helper),camera_static_validation=str(validation),
    preparation_validation=str(BASE/'outR135_corr1p5_Lstart33p7_repeat03_v8_preparation_validation.json'))
for key in ['settings_source','preset_readback','settings']:new[key]=str(session/Path(previous[key]).name)
helper.write_text(Path(previous['preflight_helper']).read_text(encoding='utf-8').replace(metas[1].name,meta_path.name),encoding='utf-8',newline='\n')
validate_files(new)
save(meta_path,new)
spec=importlib.util.spec_from_file_location('next_preflight_v8',helper); module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
import cv2
frame_path=Path(previous['session'])/'static_preflight_frame.png';frame=cv2.imread(str(frame_path));assert frame is not None
check=module.inspect_frame(frame,load(config['reference']),load(config['geometry']),plan['expected_candidate_parameters'],plan)
assert check['ready'] and check['physical_hole_clearance_verified_by_model'] is False
save(new['preparation_validation'],dict(state='PASSED_OFFLINE_FILES_AND_PREVIOUS_REAL_FRAME',hardware_commands_sent=0,
    unchanged_motion_settings=True,validated_detector_version='v8',real_frame_ready=True,
    input_sha256={str(p):sha(p) for p in [plan_path,meta_path,helper,config_path,frame_path,validation]}))
(V/'README.md').write_text('機体B・右135°出口、同一走行値2本を検出v8で再解析。\n\n前黄が照明で黄緑に映る欠測に対応し、色別hue_ranges指定を追加した。既定値は変更せず、B用v8だけ黄色25〜65を指定。33件の関連テスト成功。独立の固定288フレーム静止検査、右45出口・右135入口の走行映像と固定抽出8フレームの丸マーカー実写照合を保存。旧inR135との数値一致検査はFAILEDのまま保持：旧検出がPCB上の黄色を前マーカーと誤認する例があり、実物の丸中心との追加照合で新検出を確認した。旧inR135候補へ再測定注意を追記。\n\n今回の録画の旧解析は必要走行区間93.56%で不成立。v8では同区間100%となり、前回分99.66%とともに全フレーム・静止照合を通過。品質基準の緩和なし、走行追加なし。同一条件の独立2本として扱う。横/前後誤差は1本目+0.408/-0.803 mm、2本目+0.424/-0.488 mm、出口は-0.023/-0.040°。3本目は同じ値で確認予定。入口相対の形状評価で、盤面固定・絶対機軸・連続接続の合格ではない。\n',encoding='utf-8')
print(json.dumps(dict(next_meta=str(meta_path),valid_unique_trials=2,mean_endpoint_error_RF_mm=errors.mean(axis=0).tolist()),ensure_ascii=False))
