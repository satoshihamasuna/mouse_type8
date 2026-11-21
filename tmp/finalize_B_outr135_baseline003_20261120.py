"""出口初回の原本/色候補検証/長さ候補を対応付ける。実機操作なし。"""
from pathlib import Path
import json
import shutil
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_B_OUTR135_1600_baseline_green_20261120_003'
REC=RUN/'rear_red_s30_reanalysis_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False)


geometry=read(REC/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
report=read(REC/'full/report.json');assert geometry['independent_optical_geometry_eligible']
assert report['required_motion_window_quality']['observation_rate']>=.95
profile=BASE/'rear_red_s30_validation_v1/body_geometry_B_green35_rear_red30.json'
save(BASE/'rear_red_s30_validation_v1/full_validation.json',dict(state='FULL_CURRENT_VIDEO_QUALIFIED_FOR_RELATIVE_REVIEW',
    source=str(REC/'full/report.json'),geometry_review=str(REC/'midpoint_raw_wheel_clock_v1/midpoint_review.json'),
    required_motion_observation_rate=report['required_motion_window_quality']['observation_rate'],
    registration_rate=report['required_motion_window_quality']['calibration_rate'],threshold_unchanged=.95,
    scope='今回のB1600右135度出口と同じ撮影条件。次の独立撮影でも全区間を検証する。',
    source_original_failed_report=str(RUN/'full/report.json'),false_green_area_candidate_rejected=True))
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
stages=[]
for i in range(1,4):
    stage=BASE/f'lengths_Ls6_Le27p9_c1p9_{i:02d}'
    config_path=stage/'config.json';shutil.copyfile(config_path,stage/'config_before_rear_red_s30.json')
    config=read(config_path);config['geometry']=str(profile)
    config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    plan=read(stage/'placement_plan.json')
    payload=McpTuningService(stage/'red30_offline_validation',{}).prepare(str(config_path),str(stage/'candidate.settings.json'),plan['placement'],None,20.)
    save(stage/'offline_validation_red30.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,
        supersedes_geometry_profile_in='offline_validation.json',hardware_operations=0,
        full_profile_validation=str(BASE/'rear_red_s30_validation_v1/full_validation.json')))
    stages.append(str(stage))
result=dict(mouse='B',motion='out_r135',preset_mm_s=1600,baseline_parameters=read(RUN/'saved_1ms.settings.json')['parameters'],
    source_trial=str(RUN/'trial.json'),analysis_only_override=str(REC/'trial.json'),geometry=geometry,
    original_report=read(RUN/'full/report.json'),reanalysis_report=report,original_failure_preserved=True,
    source_false_candidate_review=read(BASE/'green_area10_validation_v1/visual_review_rejected.json'),
    profile=read(profile),operator_result=read(BASE/'baseline01/local_execution/operator_results/B_OUTR135_1600_baseline_green_20261120_003.json'),
    next_parameters=read(Path(stages[0])/'candidate.settings.json')['parameters'],next_stages=stages,
    next_request_id='B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_001',next_request_sent=False,
    source_params_changed=False,firmware_write_performed=False,power_state='BOTH_OFF_USER_CONFIRMED',
    angle_policy='実測134.786度、目標差-0.214度のため角度補正1.9度を維持し、長さのみ比較。',
    length_policy='測定時点のLend右97.984/前151.589mmと出口直線からLstart6.004/Lend27.943mmを求め、6.0/27.9mmへ丸めた比較候補。',
    adopted=False,limitations=['単体の入口相対マーカー中点。固定盤面/車軸絶対精度、BのEKF係数、独立反復・連続接続は未確認。',
        '後赤のS30は独立画像と今回全フレームで検証済み。次の撮影でも95%/各フレームArUcoと誤認を再確認。'])
save(BASE/'baseline003_red30_review.json',result)
state=read(BASE/'resume_state.json')
state.update(state='BASELINE003_RED30_REVIEWED_LENGTHS6_27P9_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED',contact_detected=False,execution_pending=False,
    baseline_review=str(BASE/'baseline003_red30_review.json'),relative_exit_rotation_deg=geometry['exit_path_direction_change_deg'],
    next_request_id=result['next_request_id'],next_request_sent=False,next_stage=stages[0],
    next_action='同じ出口180/45mm・緑前右下45度へ戻し、今回の配置/干渉なし/ON/手離し確認後、新IDで長さ6.0/27.9mmを1回比較。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as file:
    file.write('\n初回003を1回取得、操作者が両方OFF・接触なしを確認。元のマーカー観測94.304%不合格を保持。'
        '前緑の面積10px候補は背景/PCB誤認を視認し棄却。後赤の最小彩度だけ40→30とした別候補で実後マーカー8画像を確認し、'
        '独立入口3動画60画像と今回47画像で比較（旧観測点の変化最大1.467mm）。全889フレームの再解析で必要区間観測99.367%、ArUco100%を確認。'
        '元95%基準・原本・不合格・誤認例は保持。\n\n'
        '出口角134.786度、Lend右97.984/前151.589mm（目標95.459/159.099）。角度補正+1.9度を維持し、次はLstart6.0/Lend27.9mm、半径-41mm・PID/FF・吸引650維持を未実行候補として3stage準備。'
        '今回の操作者確認前に送信しない。詳細baseline003_red30_review.json。\n')
print(json.dumps(dict(exit_rotation_deg=geometry['exit_path_direction_change_deg'],next_parameters=result['next_parameters'],next_request_sent=False),ensure_ascii=False))
