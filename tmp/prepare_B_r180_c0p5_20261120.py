"""B1600右180度の角度補正だけを変更する次候補を準備。実機操作なし。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
SOURCE=BASE/'baseline01'
RUN=SOURCE/'local_execution/run_B_R180_1600_baseline_green_20261120_001'
OUT=BASE/'angle_c0p5_01'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)


actual=read(RUN/'saved_1ms.settings.json')
assert actual['mouse']=='B' and actual['motion_name']=='long_r180' and actual['preset_speed_mm_s']==1600
candidate={key:copy.deepcopy(actual[key]) for key in ('schema_version','mouse','mouse_identity_source',
    'motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')}
assert candidate['parameters']['correction']==2
candidate['parameters']['correction']=.5
candidate['source']='B1600右180度の独立実機baseline001を基に、動画入口基準の回転量178.337度を改善する比較候補。角度補正のみ+2.0から+0.5度へ変更。未送信・未実走。'
save(OUT/'candidate.settings.json',candidate)
shutil.copyfile(SOURCE/'config.green_s45.v2.json',OUT/'config.json')
shutil.copyfile(SOURCE/'static_pose_after_settle.py',OUT/'static_pose_after_settle.py')
identifier='B_R180_1600_c0p5_green_20261120_001'
code=(SOURCE/'run_recorded_once.py').read_text(encoding='utf-8')
assert code.count('B_R180_1600_baseline_green_20261120_001')==1
code=code.replace('B_R180_1600_baseline_green_20261120_001',identifier)
assert code.count("str(OUT/'config.green_s45.v2.json')")==1
code=code.replace("str(OUT/'config.green_s45.v2.json')","str(OUT/'config.json')")
compile(code,str(OUT/'run_recorded_once.py'),'exec')
with (OUT/'run_recorded_once.py').open('x',encoding='utf-8') as output:output.write(code)
plan=read(SOURCE/'placement_plan.json')
plan.update(state='CANDIDATE_PREPARED_NOT_EXECUTED',candidate=str(OUT/'candidate.settings.json'),
    config=str(OUT/'config.json'),next_unsent_request_id=identifier,
    current_placement_confirmed=True,power_state='BOTH_OFF_USER_CONFIRMED',
    current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    operator_current_readiness_confirmed=False,execution_commands_sent=0,settings_write_commands_sent=0)
save(OUT/'placement_plan.json',plan)
save(OUT/'operator_off_placement.json',dict(source='current_user_reply',
    quote='元の位置へ戻した・両方OFF',both_powered_off=True,current_placement_confirmed=True,
    planned_center_board_mm=[315,135],planned_heading_deg=90,
    current_clearance_confirmed=False,current_hands_clear_confirmed=False))
geometry=read(BASE/'baseline001_video_full_post_v2/video_comparison.json')['comparisons'][0]
tracking=read(RUN/'tracking_1ms_review_v2/verification.json')
save(OUT/'candidate_basis.json',dict(state='OFFLINE_CANDIDATE_NOT_EXECUTED',
    change=dict(field='parameters.correction',before=2.,after=.5,unit='deg'),
    measured_relative_path_rotation_deg=geometry['entry_to_exit_path_rotation_deg'],
    measured_path_angle_error_deg=geometry['path_rotation_error_from180_deg'],
    estimated_first_order_candidate_path_rotation_deg=geometry['entry_to_exit_path_rotation_deg']+1.5,
    candidate_prediction_is_unverified=True,
    exit_spacing_at_forward45_mm=geometry['exit_line_right_at_forward45_mm'],
    forward_apex_mm=geometry['forward_apex_mm'],
    terminal_longitudinal_distance_not_used_to_set_Lend=True,
    radius_and_lengths_held='角度を先に揃え、出口線間隔と頂点は角度変更後に再評価する。停止後の進み約6.95mmをLend補正に埋め込まない。',
    gyro_through_lend_measured_time_deg=tracking['gyro_through_lend_deg'],
    gyro_video_difference_note='実測周期でのジャイロ積分182.65度と、動画の入口/出口直線178.34度は基準が異なり一致しない。機首の絶対校正とBのジャイロ・滑り同定は未完了。動画で同じ基準の幾何接続を比較し、ジャイロだけで角度補正を決めない。',
    original_summary_auto_stop_confirmed=False,
    late_stationary_video_observations=geometry['stationary_observations'],
    late_stationary_scatter_rms_mm=geometry['stationary_scatter_rms_mm'],
    actual_capture_stop_confirmed=True,operator_contact_detected=False,both_powered_off=True,
    relative_review_eligible=geometry['relative_review_eligible'],
    phase_window_diagnostic=str(BASE/'baseline001_exit_window_diagnostic.json'),
    source_review=str(BASE/'baseline001_video_full_post_v2/video_comparison.json'),
    source_settings=str(RUN/'saved_1ms.settings.json'),
    source_settings_sha256=hashlib.sha256((RUN/'saved_1ms.settings.json').read_bytes()).hexdigest(),
    firmware_source_changed=False,parameter_adoption_allowed=False))
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
prepared=McpTuningService(OUT/'offline_validation',{}).prepare(str(OUT/'config.json'),
    str(OUT/'candidate.settings.json'),plan['placement'],None,20.)
save(OUT/'offline_validation.json',dict(state='VALIDATED_NOT_CONNECTED',
    parameters=prepared['settings']['parameters'],request_id=identifier,placement=plan['placement']))
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',stage=str(OUT),request_id=identifier,
    change='correction +2.0 -> +0.5 deg',hardware_commands_sent=0),ensure_ascii=False),flush=True)
