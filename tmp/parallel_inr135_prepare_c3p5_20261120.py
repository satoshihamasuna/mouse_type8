"""IN135005のsaved実設定を維持し、補正+3.5度だけの未実行候補3stageを準備する。"""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
RUN = BASE/'baseline01/local_execution/run_A_INR135_1800_baseline_20261120_005'
REVIEW = BASE/'parallel_analysis/A_INR135_1800_baseline_20261120_005'
OUT = BASE/'parallel_analysis/inr135_c3p5_angle_candidate_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
front=read(REVIEW/'front_onset_v1/front_reference.json')
tracking=read(REVIEW/'tracking_1ms_v1/verification.json')
settings=read(RUN/'saved_1ms.settings.json')
assert front['state']=='FRONT_RED_RELATIVE_REVIEW' and front['all_motion_front_quality_passed']
assert front['clock']['passed'] and settings['parameters']['correction']==0.
OUT.mkdir(exist_ok=False)
basis=dict(mouse='A',motion='in_r135',state='UNEXECUTED_ANGLE_ONLY_CANDIDATE',source_run=str(RUN),
    source_saved_settings=str(RUN/'saved_1ms.settings.json'),source_front_review=str(REVIEW/'front_onset_v1/front_reference.json'),
    source_tracking_review=str(REVIEW/'tracking_1ms_v1/verification.json'),
    changed_parameters=dict(correction=dict(before_deg=0.,candidate_deg=3.5)),
    unchanged_geometry={key:settings['parameters'][key] for key in ('r_min','lstart','lend','degree','velo','pre_accel')},
    measured_front_path_direction_deg=front['path_direction_change_deg'],angle_error_from135_deg=front['path_angle_error_deg'],
    approximate_unit_response_predicted_direction_deg=front['path_direction_change_deg']-3.5,
    prediction_is_measured_causal_model=False,
    measured_front_Lend_right_forward_mm=front['measured_front_Lend_right_forward_mm'],half_span_scenarios=front['half_span_scenarios'],
    quality=dict(observation_rate=front['observation_rate'],motion_observation_rate=front['motion_only_observation_rate'],
        entry_observation_rate=front['entry_window_observation_rate'],exit_observation_rate=front['exit_window_observation_rate'],
        entry_line=front['entry_line'],exit_line=front['exit_line'],clock=front['clock']),
    one_ms_tracking=dict(rms_rad_s=tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s'],
        saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],battery_min_v=tracking['battery_min_v']),
    preserved_settings=['saved PID','saved FF','suction650','velo1.8','pre_accel6.5','post_diagonal20','camera1500mm/EV-1.5/zoom1.8/torchfalse/focuslock'],
    marker_reference='original physical board axes/full-frame actual2/3 search',static_guard='V3 all raw target heading<=2deg; nominal planned90deg',
    length_change_deferred_reason='前点r20/25/30の前進残差が2.33/10.86/19.40mmと変わり、角度補正でも終端が動く。角度候補後に同じ赤参照で長さを再測定する。',
    future_execution='先頭stageを単体1回だけ新規確認して評価する。残り2stageは同設定の未実行予備で、自動実行しない。',
    absolute_midpoint_or_axle_claimed=False,parameter_adoption_allowed=False,hardware_operations=0,
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (RUN/'saved_1ms.settings.json',REVIEW/'front_onset_v1/front_reference.json',REVIEW/'tracking_1ms_v1/verification.json',Path(__file__))})
with (OUT/'basis.json').open('x',encoding='utf-8') as file:
    json.dump(basis,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')

# 既存CLIを使用する。baselineには任意の旧V2 snapshotがないのでコピー対象からだけ除く。
sys.path.insert(0,str(ROOT/'tmp'))
import mouse_tuning_prepare_candidate_20261120 as prepare
prepare.COPY_FILES=tuple(name for name in prepare.COPY_FILES if (BASE/'baseline01'/name).is_file())
sys.argv=[str(ROOT/'tmp/mouse_tuning_prepare_candidate_20261120.py'),
    '--base',str(BASE/'baseline01'),'--source-run',str(RUN),'--name','angle_c3p5_',
    '--id-prefix','A_INR135_1800_c3p5_angle','--output-base',str(OUT),
    '--correction','3.5','--count','3','--date','20261120','--basis',str(OUT/'basis.json')]
prepare.main()
for number in (1,2,3):
    stage=OUT/f'angle_c3p5_{number:02d}'
    plan=read(stage/'placement_plan.json')
    plan['static_apparent_reference_heading_deg']=90.
    plan['placement']['heading_deg']=90.
    plan['static_reference_basis']='Nominal planned board heading90deg. Earlier empirical89.2604deg is not used for acceptance.'
    plan['heading_acceptance_version']=3
    plan['heading_target_basis']='nominal_planned_board_heading'
    with (stage/'placement_plan.json').open('w',encoding='utf-8') as file:
        json.dump(plan,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(dict(basis=str(OUT/'basis.json'),stages=[str(OUT/f'angle_c3p5_{number:02d}') for number in (1,2,3)],physical_execution=False),ensure_ascii=False),flush=True)
