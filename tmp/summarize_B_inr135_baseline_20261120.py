"""B1600右135度入口の初回・未実行要求・次候補の根拠を保存する。機器操作なし。"""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
SESSION=BASE/'baseline01/local_execution'
RID='B_INR135_1600_baseline_green_20261120_002'
RUN=SESSION/('run_'+RID)
REC=RUN/'recovered_metadata_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False)


failed=read(SESSION/'jobs/B_INR135_1600_baseline_green_20261120_001.json')['result']
assert failed['execution_write_attempted'] is False and failed['ble_disconnected'] is True
save(BASE/'baseline001_unexecuted_review.json',dict(request_id='B_INR135_1600_baseline_green_20261120_001',
    state='NOT_EXECUTED_BLE_DISCONNECTED_BEFORE_DEVICE_CHECK',execution_write_attempted=False,
    settings_write_commands_sent=0,preset_selections_sent=0,execution_resend_count=0,
    streaming_rows=798,stream_lost=0,stream_crc_errors=0,ideal_velocity_and_angular_velocity_all_zero=True,
    low_battery_rejection=False,ble_disconnected=True,error=failed['error'],
    competing_ble_processes_found_at_followup=False,physical_disconnect_cause_identified=False,
    original_evidence_retained=True,power_cycle_source=str(BASE/'operator_ready_baseline002.json')))
review=read(REC/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
tracking=read(RUN/'tracking_1ms_review_v3/verification.json')
candidate=read(BASE/'angle_cm0p4_01/candidate.settings.json')
assert candidate['parameters']['correction']==-.4 and review['independent_optical_geometry_eligible']
result=dict(mouse='B',motion='in_r135',preset_mm_s=1600,request_id=RID,
    source_trial=str(RUN/'trial.json'),recovered_trial=str(REC/'trial.json'),
    archive=str(read(REC/'archive.json')['archive_folder']) if 'archive_folder' in read(REC/'archive.json') else read(REC/'archive.json'),
    operator_result=read(SESSION/'operator_results'/f'{RID}.json'),
    tracking=tracking,geometry=review,next_stage=str(BASE/'angle_cm0p4_01'),
    next_request_id='B_INR135_1600_cm0p4_green_20261120_001',next_request_sent=False,
    candidate_change='角度補正のみ+2.1から-0.4度へ変更。右旋回は補正を減らすと回転量が増える。',
    candidate_basis_rotation_deficit_deg=135-review['exit_path_direction_change_deg'],
    lengths_preserved_for_angle_comparison=True,source_params_changed=False,firmware_write_performed=False,
    EKF=dict(status='NOT_IDENTIFIED',reason='Bの機軸/車軸幾何未校正・解析ツールのA制約。今回の幾何候補からKや雑音を推測しない。'),
    software_fix=dict(file='tools/pixel_camera_tuning.py',change='参照/機体JSONのsnapshotとtrialのvalueを分離',
        regression_failed_before_fix=True,tests_passed={'test_pixel_camera_tuning.py':21,'test_pixel_camera_turn_capture.py':26,'test_pixel_camera_tuning_mcp.py':7},
        gui_test_requires_normal_python_environment=True),
    original_auto_stop_summary_retained=read(REC/'full/summary.json'),
    limitations=['単発の相対形状。固定盤面接続・車軸姿勢の絶対精度・再現性・連続接続は未合格。',
        '初期直進の高速窓が不足する既定summaryの失敗を保持。独立に入口24点/76.31mmで方向を確認。',
        '元trial情報の上書き不具合は解析用の別索引へ出所付きで復元し、原本の失敗を保持。'])
save(BASE/'baseline002_review.json',result)
state=read(BASE/'resume_state.json')
state.update(state='BASELINE002_REVIEWED_ANGLE_MINUS0P4_PREPARED_BOTH_OFF',
    next_request_id=result['next_request_id'],next_request_sent=False,next_stage=result['next_stage'],
    baseline_review=str(BASE/'baseline002_review.json'),relative_exit_rotation_deg=review['exit_path_direction_change_deg'],
    next_action='角度補正-0.4度のみ比較。元315/135mm・緑前下向きへ戻し、現在の干渉なし・両方ON・手離し確認後に新IDで1回。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as file:
    file.write('\n初回001は静止配置5/5通過後にBLE切断、実機照会・設定・走行は0件。新規002で1回取得、操作者は両方OFF・接触なしを確認。'
        '保存ログ228点/drop0、実測周期1.015521ms、角速度RMS2.886rad/s、飽和0%、最低7.418V。\n\n'
        '全フレームの必要走行区間は観測/ArUco登録99.346%。入口raw wheelだけで時刻を合わせ、入口相対の回転量132.476度、'
        'Lend中点は右92.014/前130.396mm（目標90/135）。次は補正+2.10→-0.40度のみ変更する未実行候補。長さは角度比較後に再評価。\n\n'
        '保存ツールのtrial変数上書きを修正し、再現テストを追加。21+26+7テストを確認。今回原本は変更せずrecovered_metadata_v1へ'
        '実機設定・送信イベントを出所として別索引を復元し、検証済みの同一動画キャッシュから再解析した。'
        '既定summaryの初期直進窓不足は保持、後半静止105点のばらつき0.163mmと操作者確認を別に記録。BのEKF係数は未同定。\n')
print('Saved baseline review, preserved failed request, and prepared next angle comparison')
