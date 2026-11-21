"""右out135の同設定3本をまとめる。採用・追加走行・実機書込みは行わない。"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import math
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'standard_flow_c1p0_Ls5p3_Le31_01'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_standard_green_20261121_001'
run=stage/'local_execution'/('run_'+identifier)
out=base/'geometry_same_candidate_three_review_v1';assert not out.exists()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
previous_path=base/'geometry_same_candidate_two_review_v1/review.json';previous=read(previous_path)
rows=copy.deepcopy(previous['runs'])
for row in rows:assert sha(Path(row['geometry_review']))==row['geometry_review_sha256']
geometry_path=run/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
geometry=read(geometry_path);report=read(run/'full/report.json');trial=read(run/'trial.json')
tracking=read(run/'tracking_1ms_review_v3/verification.json')
settings=read(run/'saved_1ms.settings.json')
operator=read(run.parent/'operator_results'/(identifier+'.json'))
job=read(run.parent/'jobs'/(identifier+'.json'))
assert geometry['independent_optical_geometry_eligible'] and geometry['clock']['passed']
assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
assert operator['both_powered_off'] and operator['contact_detected'] is False
assert tracking['execution_resend_count']==0 and trial['recording_stop']['log_stop_evidence']['confirmed']
assert job['request']['config']['address']=='E1:7F:86:21:84:73' and trial['stream_period_ms']==5
assert {key:settings[key] for key in previous['settings']}==previous['settings']
runtime=read(stage/'runtime_conditions_for_review.json')
assert {key:value for key,value in runtime.items() if key!='sources'}==previous['same_runtime_conditions']
first_analysis=Path(rows[0]['analysis_trial']).parent
assert read(first_analysis/'body_geometry_used.json')==read(run/'body_geometry_used.json')
samples=read(run/'marker_visual_check_v1/samples.json');assert len(samples['samples'])==16
save(run/'marker_visual_check_v1/visual_review.json',dict(state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED',
    inspected_image='actual_marker_pairs.jpg',samples=16,observation='入口・旋回・出口の16画像で実前緑と後赤/桃色マーカーへの重なりを確認。',
    whole_video_human_review_performed=False,acceptance_thresholds_unchanged=True))
point=geometry['midpoint_Lend_right_forward_mm'];target=geometry['target_midpoint_Lend_right_forward_mm']
stop=geometry['late_stationary']['midpoint_right_forward_mm'];stop_target=[112.5*math.sqrt(2),67.5*math.sqrt(2)]
angle=geometry['exit_path_direction_change_deg']
rows.append(dict(request_id=identifier,actual_trial=str(run/'trial.json'),analysis_trial=str(run/'trial.json'),
    geometry_review=str(geometry_path),geometry_review_sha256=sha(geometry_path),exit_angle_deg=angle,
    Lend_right_forward_mm=point,Lend_error_mm=math.dist(point,target),
    Lstart_delta_from_exit_line_mm=geometry['Lstart_delta_from_exit_line_mm'],
    exit_line_transverse_offset_mm=geometry['Lstart_delta_from_exit_line_mm']*math.sin(math.radians(angle)),
    stop_right_forward_mm=stop,stop_error_mm=math.dist(stop,stop_target),Lend_to_stop_mm=math.dist(point,stop),
    required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
    registration_fraction=report['required_motion_window_quality']['calibration_rate'],
    tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],saturation_pct=tracking['tracking']['saturation_pct'],
    battery_min_v=tracking['battery_min_v'],clock_fit_rms_mm=geometry['clock']['incoming_raw_wheel_fit_rms_mm'],
    clock_half_spread_ms=geometry['clock']['half_window_zero_spread_ms'],operator_result=operator,
    capture_stop_evidence=trial['recording_stop']['log_stop_evidence']))
assert len({row['request_id'] for row in rows})==3
assert all(row['required_observation_fraction']>=.95 and row['registration_fraction']==1 for row in rows)
mean_point=[statistics.mean(row['Lend_right_forward_mm'][axis] for row in rows) for axis in (0,1)]
stats=dict(mean_exit_angle_deg=statistics.mean(row['exit_angle_deg'] for row in rows),
    angle_population_sd_deg=statistics.pstdev(row['exit_angle_deg'] for row in rows),
    angle_range_deg=max(row['exit_angle_deg'] for row in rows)-min(row['exit_angle_deg'] for row in rows),
    maximum_absolute_angle_error_deg=max(abs(row['exit_angle_deg']-135) for row in rows),
    mean_Lend_right_forward_mm=mean_point,mean_Lend_point_error_mm=math.dist(mean_point,target),
    maximum_Lend_error_mm=max(row['Lend_error_mm'] for row in rows),
    maximum_abs_exit_line_transverse_offset_mm=max(abs(row['exit_line_transverse_offset_mm']) for row in rows),
    stop_error_range_mm=[min(row['stop_error_mm'] for row in rows),max(row['stop_error_mm'] for row in rows)],
    observed_Lend_to_stop_range_mm=[min(row['Lend_to_stop_mm'] for row in rows),max(row['Lend_to_stop_mm'] for row in rows)])
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),state='THREE_RUN_RELATIVE_SINGLE_TURN_CANDIDATE',
    mouse='B',motion='out_r135',preset_mm_s=1600,successful_same_candidate_runs=3,settings=previous['settings'],
    same_runtime_conditions=previous['same_runtime_conditions'],runs=rows,statistics=stats,
    previous_comparison=str(previous_path),previous_comparison_sha256=sha(previous_path),
    source_profile=str(base/'rear_hue165_validation_v2/full_validation.json'),
    reference='各試行の実測入口基準の高さ補正前後マーカー中点。固定盤面/絶対車軸/機軸ではない。',
    first_original_failure_preserved=True,reanalysis_counted_as_same_physical_run=True,
    failed_BLE_requests_excluded=['green_20261121_002','repeat_green_20261121_001','repeat_green_20261121_002'],
    standard_capture_followup=dict(successful_captures=1,attempts=1,stream_period_ms=5,
        source_trial=str(run/'trial.json'),root_cause_proven=False,
        note='独自perform除去後の今回1回は取得完了。反復/長期の通信改善や切断原因の確定とはしない。'),
    communication_only_review=str(base/'communication_only_v1/review.json'),
    power_state='BOTH_OFF_USER_CONFIRMED',contact_detected=False,adopted=False,
    firmware_parameter_source_changed=False,firmware_write_performed=False,
    further_run_automatically_scheduled=False,
    limitations=['3本の相対形状候補。固定盤面/絶対車軸・機軸、独立ハード同期は未校正。',
        '停止点の約8mm差と、指令90mmに対し観測約96mmの後走を、ターンの出口形状と別に保持する。',
        '左135度出口、連続走行の接続、BのEKF係数の同定は未確認。',
        '元の公式summaryと採用制限は保持し、数値をゼロに合わせた新しい許容基準は設けない。'])
out.mkdir();save(out/'review.json',result)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6,7),layout='constrained')
for number,row in enumerate(rows,1):
    folder=Path(row['analysis_trial']).parent
    pose=pd.read_csv(folder/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    valid=pose.registered_midpoint_valid.astype(bool)
    ax.plot(pose.right_mm.where(valid),pose.forward_mm.where(valid),label=f'{number}本目 {row["exit_angle_deg"]:.3f}°')
    ax.scatter(*row['Lend_right_forward_mm']);ax.scatter(*row['stop_right_forward_mm'],marker='+')
ax.scatter(*target,marker='x',color='black',label='Lend目標')
ax.scatter(*stop_target,marker='s',facecolors='none',edgecolors='black',label='停止目標（別評価）')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='機体B1600 右135度出口 / 同設定3本')
ax.grid(alpha=.3);ax.legend(fontsize=8);fig.savefig(out/'three_trajectories.png',dpi=140);plt.close(fig)
text='# 機体B1600 右135度出口・同設定3本の単体候補\n\n'
text+='候補：**速度1.6 m/s、半径−41 mm、Lstart5.3 mm、Lend31.0 mm、角度補正+1.0°**。B PID/FF、吸引ON duty650、前走加速度指定6.5 m/s²、斜め前走127.279 mm・直進後走90 mm・後走加速度20 m/s²、各実機モードを揃えた。B用BLE E1:7F:86:21:84:73。前緑・後赤/桃色、検出条件も全解析で共通。\n\n'
text+='|試行|入口相対出口角 [deg]|Lend右/前 [mm]|Lend目標差 [mm]|出口線横ずれ [mm]|停止点目標差 [mm]|必要区間観測 [%]|\n|---|---:|---:|---:|---:|---:|---:|\n'
for number,row in enumerate(rows,1):
    text+=f'|{number}|{row["exit_angle_deg"]:.3f}|{row["Lend_right_forward_mm"][0]:.3f}/{row["Lend_right_forward_mm"][1]:.3f}|{row["Lend_error_mm"]:.3f}|{row["exit_line_transverse_offset_mm"]:.3f}|{row["stop_error_mm"]:.3f}|{row["required_observation_fraction"]*100:.3f}|\n'
text+=f'\n平均角 **{stats["mean_exit_angle_deg"]:.3f}°**、角度の最大目標差{stats["maximum_absolute_angle_error_deg"]:.3f}°、Lend目標差の最大{stats["maximum_Lend_error_mm"]:.3f} mm。平均Lendの目標差{stats["mean_Lend_point_error_mm"]:.3f} mm。毎評価フレームArUco登録100%、観測95%以上、3本とも走行中欠落/CRC0・出力飽和0、操作者が接触なし・両方OFFを確認した。\n\n'
text+=f'停止点差は{stats["stop_error_range_mm"][0]:.3f}〜{stats["stop_error_range_mm"][1]:.3f} mm、Lendから停止までの観測中点移動は{stats["observed_Lend_to_stop_range_mm"][0]:.3f}〜{stats["observed_Lend_to_stop_range_mm"][1]:.3f} mm。指令の後走90 mmとの違いを残し、出口長へ一律に埋め込まない。停止精度・連続接続の合格ではない。\n\n'
text+='![入口相対の3軌跡](three_trajectories.png)\n\n'
text+='1本目の元検出91.875%不合格は保持。独立画像と実マーカー視認で検証した後赤色相165..179の別解析を、同じ物理走行の評価として用いた。BLE失敗3要求は走行指令0件で、本数に含めていない。\n\n3本目は標準MCP手順で1回取得できた。停止中の通信試験20/10/5 ms各20秒も通過しているが、独自前処理の除去が切断原因の解消だったとは断定しない。元のsummary判定を維持する。\n\n右の単体形状候補として記録し、追加走行は自動で予定しない。**出口値のソース反映・書込み、左出口、連続接続、BのEKF係数は未実施/未確認**。前後マーカー中点の実測入口相対評価であり、絶対車軸/機軸/盤面固定精度の保証ではない。根拠は[review.json](review.json)。\n'
(out/'README.md').write_text(text,encoding='utf-8')
save(run/'same_candidate_three_review.json',result)
phase=read(stage/'phase.json');phase.update(state='CAPTURED_RELATIVE_REVIEWED_THIRD_CANDIDATE_RECORDED',
    next_candidate_executed=True,execution_pending=False,successful_run_count=1,review=str(out/'review.json'))
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
state.update(state='THREE_RUN_RELATIVE_CANDIDATE_RECORDED_BOTH_OFF',power_state='BOTH_OFF_USER_CONFIRMED',
    operator_ready=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    execution_pending=False,ble_connected=False,successful_same_candidate_runs=3,
    last_request_id=identifier,last_request_result='CAPTURED_RELATIVE_REVIEWED',last_run=str(run),
    last_execution_write_attempted=True,execution_commands_sent=1,last_error=None,
    relative_exit_rotation_deg=angle,position_error_mm=rows[-1]['Lend_error_mm'],
    video_required_observation_fraction=rows[-1]['required_observation_fraction'],video_registration_fraction=1.,
    comparison_review=str(out/'review.json'),candidate_status=result['state'],
    next_stage=None,next_request_id=None,next_request_sent=False,next_action_kind='candidate_review_no_automatic_run',
    next_action='右out135の同設定3本の相対形状候補を提示。停止点差/未確認の左/連続接続/ソース未反映を区別する。追加走行や反映を自動実行しない。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write(f'\n標準手順standard_001を1回取得し、同設定の成功走行3本が揃った。3本目{angle:.3f}度/Lend目標差{rows[-1]["Lend_error_mm"]:.3f}mm、必要区間観測98.750%/ArUco100%、1ms279点/drop0、走行中欠落/CRC0・飽和0、両方OFF・接触なし。3本平均角{stats["mean_exit_angle_deg"]:.3f}度、Lend差最大{stats["maximum_Lend_error_mm"]:.3f}mm。停止点差は別評価で保持。出口値のソース反映/書込み、左出口、連続接続、BのEKF係数は未確認。追加走行は自動予定せず、候補と根拠をgeometry_same_candidate_three_review_v1/README.mdへ保存。\n')
print(json.dumps(dict(statistics=stats,current=rows[-1],state=result['state']),ensure_ascii=False),flush=True)
