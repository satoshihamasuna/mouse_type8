"""操作者指定の概略配置・動画入口基準へ切り替える。既存要求は再送しない。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
OLD=BASE/'baseline01'
OUT=BASE/'baseline02_relative'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
oldid='B_INR45_1600_baseline_green_20261120_001'
identifier='B_INR45_1600_relative_green_20261120_002'
job=read(OLD/'local_execution/jobs'/f'{oldid}.json')
assert job['state']=='FAILED' and job['event']=='static_check_before_ble'
assert not (OLD/'local_execution'/('run_'+oldid)).exists()
for name in ('candidate.settings.json','config.json','placement_plan.json'):
    shutil.copyfile(OLD/name,OUT/name)
static=OLD/'local_execution'/('camera_before_ble_'+oldid)/'static_check.json'
strict=read(static)
assert strict['observed_fraction']>=.95 and strict['calibration']['calibration_frame_rate']>=.95
assert strict['calibration']['per_frame_registration_required']
assert strict['max_position_error_mm']<=10 and strict['representative_heading_error_deg']<=5
assert strict['heading_jitter_max_deg']<=5
checked=dict(strict,passed=True,acceptance_scope='user_requested_coarse_setup_then_measured_entry_relative_review',
    original_strict_static_result=str(static),original_strict_passed=strict['passed'],
    original_strict_failure_reasons=strict['failure_reasons'],failure_reasons=[],
    precise_static_pose_gate_used=False,precise_static_pose_quality_passed=False,
    user_requested_relative_entry_alignment=True,
    current_operator_confirmed_unchanged_position=True,required_motion_quality_unchanged=True,
    coarse_acceptance_limits=dict(max_position_error_mm=10.,median_heading_error_deg=5.,
        maximum_median_heading_deviation_deg=5.,minimum_observation_and_registration_fraction=.95),
    acceptance_reason='ユーザー「おおよその位置で動画で補正してほしい」。現在位置不変・両方ON・手離し・1回走行を再確認。精密静止角条件は採用せず、99コマの実観測・各コマArUco登録と概略配置を使用。走行動画の品質条件は変更しない。')
save(OUT/'relative_setup_check.json',checked)
save(OUT/f'operator_ready_{identifier}.json',dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),
    request_id=identifier,operator_answer='両方ON・手離し済み・1回走行可',
    evaluation_request='おおよその位置で動画で補正してほしい',
    unchanged_placement=True,both_powered_on=True,hands_clear=True,authorized_executions=1,
    prior_request_execution_commands_sent=0))
text=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
start,end=text.index('def dense_static_check('),text.index('tuning.static_check = dense_static_check')
replacement='''def dense_static_check(camera, device, folder, placement):
    # 新規録画ではなく、操作者が位置不変を確認した直前の実測静止資料を使う。
    checked = tuning.read_json(OUT/'relative_setup_check.json')
    if placement != checked['placement'] or checked['passed'] is not True:
        raise RuntimeError('現在の配置計画と概略確認資料が一致しません')
    tuning.save_json(folder/'static_check.json', checked)
    return checked


'''
text=text[:start]+replacement+text[end:]
text=text.replace(oldid,identifier).replace('current strict static check and measured-entry-relative review',
    'user requested approximate setup and video measured-entry-relative correction')
(OUT/'run_recorded_once.py').write_text(text,encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state='USER_REQUESTED_RELATIVE_SETUP_READY',stage=str(OUT),request_id=identifier,
    request_sent=False,execution_pending=True,operator_ready=True,
    physical_power_state='BOTH_ON_HANDS_CLEAR_USER_CONFIRMED',
    prior_request=dict(id=oldid,state='FAILED_STATIC_ONLY',execution_commands_sent=0),
    approximate_setup_explicitly_authorized=True,strict_static_result_preserved=str(static))
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
    f.write('\n初回001は精密静止角のRMS0.564度/半窓差0.586度で停止。位置225.79/46.64mm・角度90.84度、前後と実ArUco99/99コマ。BLE接続・走行指令0件。ユーザー指定「おおよその位置で動画で補正してほしい」により概略配置と入口相対評価へ変更。直前画像と位置不変・両方ON・手離しを再確認し、別stage/ID002で一回取得。元の精密判定は失敗として保持し、走行動画の各フレーム品質条件は維持。\n')
print(OUT)
