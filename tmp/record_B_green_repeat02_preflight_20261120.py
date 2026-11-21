"""緑反復2本目の静止確認の範囲を記録する。機器操作なし。"""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


state = read(BASE/'resume_state.json')
assert not state['execution_pending']
check = read(BASE/'green_Lstart13p6_repeat02_placed02_static_check.json')
assert check['observed_fraction'] >= .95
qa = dict(mouse='B', motion='long_r90', front_colour='green', rear_colour='red',
    source_static_check='green_Lstart13p6_repeat02_placed02_static_check.json',
    strict_static_alignment_passed=check['passed'],
    strict_static_failure_reasons=check['failure_reasons'],
    static_center_board_mm=check['center_board_mm'], static_apparent_heading_deg=check['heading_deg'],
    static_heading_jitter_rms_deg=check['heading_jitter_rms_deg'],
    actual_round_marker_identity_fixed10_manually_confirmed=True,
    body_and_marker_manual_review='静止10点全て前の緑円・後ろの赤円を指している。白い基板部分や格子を前マーカーと誤認していない。',
    evaluation_frame='relative_to_measured_motion_entry_direction',
    fixed_board_alignment_accepted=False, absolute_axle_position_accepted=False,
    static_heading_not_used_as_motion_entry_axis=True,
    diagnosis='彩度閾値20/15への変更は固定10点×3動画で一貫した改善がなく未適用。露出-2 EVも静止の向き安定性を解決せず未採用。',
    camera_ev_for_next_trial=-1.5, detection_profile_changed=False,
    motion_quality_will_be_checked_again=True,
    relative_motion_quality_policy='従来と同じ全フレームの実登録・生検出率95%以上、対応軌跡RMSE8mm以下を確認する。厳密な盤面固定静止判定の失敗は保持する。',
    device_settings_source=str(BASE/'device_prepare_green_Lstart13p6_01/actual.settings.json'),
    operator_charged_both_off_hands_clear=True,
    operator_clearance_and_ON='pending fresh reply',
    robot_execution_commands_sent=0, execution_resend_count=0,
    parameter_adoption_allowed=False)
with (BASE/'preflight_green_Lstart13p6_repeat02.json').open('x',encoding='utf-8') as output:
    json.dump(qa,output,ensure_ascii=False,indent=2)
state.update(state='GREEN_REPEAT02_OFF_STATIC_RELATIVE_SCOPE_REVIEWED_WAIT_ON',
    power_state='BOTH_OFF_USER_CONFIRMED', charging_completion_confirmed=True,
    static_report=str(BASE/'green_Lstart13p6_repeat02_placed02_static_check.json'),
    static_passed=False, static_failure_reasons=check['failure_reasons'],
    static_xy_mm=check['center_board_mm'], static_heading_deg=check['heading_deg'],
    static_heading_budget_deg=check['heading_budget_deg'],
    static_observed_frames=check['observed'], static_total_frames=check['frames'],
    current_position='Registered start vicinity after user adjusted direction; both OFF and hands clear confirmed.',
    current_placement_confirmed=True, current_hands_clear_confirmed=True,
    relative_setup_review=str(BASE/'preflight_green_Lstart13p6_repeat02.json'),
    config=str(BASE/'config.green_front.v1.json'),
    next_action='配置・向きは維持。経路の全接地点・車幅・停止余裕に干渉なし、UART配線、両方ON・手離しの新規返答後に実機設定を復元照合する。',
    execution_pending=False, pending_request_id=None, pending_request_sent=False,
    next_unsent_request_id='B_R90_1600_Lstart13p6_green_20261120_002')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(strict_alignment_passed=False, relative_scope_review_saved=True,
    device_settings_ready=True, next_execution_sent=False),ensure_ascii=False))
