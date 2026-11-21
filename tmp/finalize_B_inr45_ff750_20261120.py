"""B1600 inR45のFF比較を整理し、Iゲイン比較の再開条件を残す。"""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'ff_om_accdec0p00075_01/local_execution/run_B_INR45_1600_ffaccdec750_green_20261120_003'
NEXT=BASE/'ff750_omki0p005_01'
OUT=BASE/'ff750_comparison_v1'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
rows=[]
for name,folder in [('基準FF0.000378',BASE/'baseline02_relative/local_execution/run_B_INR45_1600_relative_green_20261120_002'),
                    ('加減速FF0.000750',RUN)]:
    track=read(folder/'tracking_1ms_review_v3/verification.json')
    geom=read(folder/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
    assert geom['independent_optical_geometry_eligible']
    assert read(folder/'operator_after_run.json')['both_powered_off']
    rows.append(dict(name=name,source_trial=str(folder/'trial.json'),settings=read(folder/'saved_1ms.settings.json'),
        tracking=track['tracking'],measured_log_period_ms=track['measured_log_period_ms'],
        exit_path_deg=geom['exit_path_direction_change_deg'],exit_error_deg=geom['exit_path_direction_change_deg']-45.,
        Lend_right_forward_mm=geom['midpoint_Lend_right_forward_mm'],Lend_residual_mm=geom['midpoint_Lend_residual_mm'],
        required_motion_quality=geom['independent_clock_required_motion_quality'],clock=geom['clock'],
        late_stationary=geom['late_stationary'],saved_samples=track['sample_count'],battery_min_v=track['battery_min_v']))
report=dict(state='FF_COMPARISON_REVIEWED_ANGULAR_I_COMPARISON_PENDING',mouse='B',motion='in_r45',preset_mm_s=1600,
    rows=rows,next_stage=str(NEXT),next_request_id=read(NEXT/'placement_plan.json')['request_id'],
    next_settings=read(NEXT/'candidate.settings.json'),next_request_sent=False,operator_ready=False,
    source_params_changed=False,firmware_written=False,physical_power_state='BOTH_OFF_USER_CONFIRMED',
    decision='角加減速FF0.000750で旋回中RMSと出口角誤差は減少したが、ピークと後揺れは改善せず。採用確定をせず、Iゲイン0.010→0.005のみ次に比較する。',
    original_summary=read(RUN/'full/summary.json'),
    automatic_stop_acceptance_replaced=False,
    stop_evidence=dict(capture_controller=read(RUN/'trial.json')['recording_stop'],
        optical_late_stationary=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')['late_stationary'],
        operator=read(RUN/'operator_after_run.json')),
    evaluation_scope='実測入口の位置/向きを補正したマーカー中点。前走90mmを含むLend目標は右45/前180mm。各評価コマで登録済みArUco2個以上を実復号。',
    limitations=['各条件一回のみ。独立反復・左右対称性・連続接続・ファーム反映は未確認。',
        'BのEKF係数は未同定。絶対車軸位置・真のbody yaw・独立ハードウェア同期は未検証。',
        '元summaryのauto_stop=falseを保持し、撮影時停止確認・後半静止動画・操作者確認を別に記録。'])
save(OUT/'review.json',report)
text='# 機体B turn1600 右in45：角加減速FF比較\n\n'
text+='入口相対の動画評価。各条件は単発で、採用確定はしていません。\n\n'
text+='| 条件 | 角速度RMS rad/s | 実測ピーク rad/s | 出口角 度 | Lend差 右/前 mm | 後揺れ 負/正 rad/s |\n|---|---:|---:|---:|---|---|\n'
for row in rows:
    m=row['tracking'];r=row['Lend_residual_mm']
    text+=f"| {row['name']} | {m['tracking_rms_rad_s']:.3f} | {m['ego_peak_rad_s']:.3f} | {row['exit_path_deg']:.3f} | {r[0]:+.3f}/{r[1]:+.3f} | {m['post_peak_neg_rad_s']:.3f}/{m['post_peak_pos_rad_s']:.3f} |\n"
text+='\nFF0.000750で旋回中RMSと出口角誤差は減りましたが、ピークと旋回直後の逆向きの揺れが残ります。'
text+='次は角速度Iゲインのみ0.010→0.005へ下げて一回比較します。FF、P/D、形状、吸引650は保持。'
text+='未実行で、元225/45mm・前緑下向きへの概略配置と両方ON・手離しの新しい確認が必要です。\n\n'
text+='全フレームの必要動作区間で前後観測は約98.6%、実ArUco登録100%。各試行の時刻対応は入口raw wheel距離だけで照合し、出口や旋回を時刻/尺度合わせに使いません。'
text+='元summaryのauto_stop=falseは保持。撮影制御の1.412秒静止、後半動画105コマのscatter0.374mm、操作者の両方OFF・接触なしを別根拠にします。'
text+='ファーム/Params変更なし。BのEKF係数は未同定です。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
pending=read(RUN/'deferred_analysis.json')
pending.update(state='FULL_ANALYZED_REVIEW_REQUIRED',full_returncode=0,quick_skipped='全フレームを明示窓で評価済み。速報は重複実行しない。',
    selected_geometry_review=str(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),archive_state='PENDING')
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state=report['state'],physical_power_state=report['physical_power_state'],execution_pending=False,
    execution_commands_sent=2,next_stage=str(NEXT),next_settings=str(NEXT/'candidate.settings.json'),
    next_request_id=report['next_request_id'],next_request_sent=False,next_operator_ready=False,
    latest_review=str(OUT/'review.json'),
    next_action='両方OFFで元225/45mm・緑前下向きへ概略配置。干渉なし・両方ON・手離しを確認後、新IDでIゲインだけ変更して一回比較。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n角加減速FF0.000750の単発比較003を取得。旋回中RMS7.268→5.596rad/s、出口角48.777→46.827度、Lend差は右+4.609/前+0.293mm。後揺れ負ピークは-9.203→-12.727rad/sで悪化したため採用確定せず、次はIゲインのみ0.010→0.005で比較。詳細はff750_comparison_v1/README.md。両方OFF・接触なし確認済み。\n')
print(OUT)
