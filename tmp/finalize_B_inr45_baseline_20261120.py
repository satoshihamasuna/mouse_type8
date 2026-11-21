"""B1600 inR45の基準走行と未実行FF比較候補を整理する。機器操作なし。"""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'baseline02_relative/local_execution/run_B_INR45_1600_relative_green_20261120_002'
NEXT=BASE/'ff_om_accdec0p00075_01'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
geometry=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
tracking=read(RUN/'tracking_1ms_review_v3/verification.json')
assert geometry['target_midpoint_Lend_right_forward_mm']==[45.,180.]
assert geometry['independent_optical_geometry_eligible']
assert read(RUN/'operator_after_run.json')['both_powered_off']
save(RUN/'midpoint_review_selection.json',dict(selected='midpoint_raw_wheel_clock_v3',
    invalid_v1='動画原点は前走開始だが目標の前進量に前走90mmを加え忘れた。候補採用へ使わない。',
    superseded_v2='数値目標は訂正済みだが図のタイトルが135度のまま。v3を使用。',
    target_basis='前走90mm + in45単体の右45/前90mm = 右45/前180mm。位置原点は実測前走開始。'))
report=dict(state='BASELINE_REVIEWED_FF_COMPARISON_PENDING',mouse='B',motion='in_r45',preset_mm_s=1600,
    source_trial=str(RUN/'trial.json'),settings=read(RUN/'saved_1ms.settings.json'),
    geometry=geometry,tracking=tracking,operator_result=read(RUN/'operator_after_run.json'),
    original_full_summary=read(RUN/'full/summary.json'),
    stop_evidence_note='元summaryの初期直進観測不足によるauto_stop=falseを保持。撮影制御の1230ms静止・後半動画105コマscatter0.268mm・操作者確認を別に示す。',
    next_stage=str(NEXT),next_request_id=read(NEXT/'placement_plan.json')['request_id'],
    next_settings=read(NEXT/'candidate.settings.json'),next_request_sent=False,
    physical_power_state='BOTH_OFF_USER_CONFIRMED',source_params_changed=False,firmware_written=False,
    turn_EKF='Bの機体幾何/校正と解析対応が未完了。K・雑音は未同定/未反映。',
    review_commands=['tools/analysis/pixel_camera_tuning_report.py --trial <source_trial> --stride 1',
        'tmp/review_B_inr45_tracking_20261120.py baseline02_relative/local_execution/run_B_INR45_1600_relative_green_20261120_002',
        'tmp/review_B_inr45_midpoint_20261120.py baseline02_relative/local_execution/run_B_INR45_1600_relative_green_20261120_002'],
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'telemetry.csv',
        RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json',RUN/'tracking_1ms_review_v3/verification.json')})
save(BASE/'baseline_review.json',report)
pending=read(RUN/'deferred_analysis.json')
pending.update(state='FULL_ANALYZED_REVIEW_REQUIRED',full_returncode=1,quick_skipped='全フレーム評価を先に完了。速報は重複実行しない。',
    selected_geometry_review=str(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),
    original_summary_preserved=True,geometry_independently_eligible=True,archive_state='PENDING')
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state=report['state'],physical_power_state=report['physical_power_state'],
    execution_commands_sent=1,request_sent=True,execution_pending=False,
    next_stage=str(NEXT),next_request_id=report['next_request_id'],next_request_sent=False,
    next_settings=str(NEXT/'candidate.settings.json'),next_operator_ready=False,
    next_action='両方OFFで元225/45mm・緑前下向きへ概略配置。干渉なし・両方ON・手離しの新規確認後、新IDでFFのみ変更して一回比較。',
    latest_review=str(BASE/'baseline_review.json'))
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
    f.write('\n基準002を一回取得し、両方OFF・接触なし確認。保存1msログ192点、drop0・実測周期1.016476ms・飽和0%。'
        '角速度RMS7.268rad/s、目標ピーク29.094に対し実測36.625rad/s。'
        '全フレームの動作必須区間は前後観測98.61%・各コマArUco登録100%。入口raw wheelだけで時計を合わせ、'
        '入口相対の出口方向48.777度、Lend中点は右50.964/前180.756mm。目標45/180mmとの差は右+5.964/前+0.756mm。'
        '前走90mmを含む目標を使うmidpoint_raw_wheel_clock_v3が正本。v1の前走加算漏れ、v2の図タイトル誤記は保持して採用対象外とした。\n\n'
        '次の未実行比較候補は角加減速FFを共通に0.000378→0.000750。PID・形状・吸引は保持。'
        '単発閉ループ推定の約0.00110は各相の観測不足警告があり確定値にしない。追従比較後に角度と長さを再評価する。'
        '元summaryのauto_stop=falseを保持し、撮影制御・後半静止像・操作者確認を別根拠に記録。ファーム/Params変更なし。\n')
print(json.dumps(dict(exit_deg=geometry['exit_path_direction_change_deg'],
    Lend_residual_mm=geometry['midpoint_Lend_residual_mm'],next_stage=str(NEXT)),ensure_ascii=False))
