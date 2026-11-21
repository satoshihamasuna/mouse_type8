"""B1600右135度出口の同設定2本を比較し、3本目の再開条件を保存する。"""
from pathlib import Path
from datetime import datetime, timezone
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
first=base/'geometry_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'
current=base/'geometry_c1p0_Ls5p3_Le31_03/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_003'
pairs=[(first,first.parent/(first.name+'_reanalysis_hue165_v1')),(current,current)]
out=base/'geometry_same_candidate_two_review_v1';assert not out.exists()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
samples=read(current/'marker_visual_check_v1/samples.json');assert len(samples['samples'])==16
visual_path=current/'marker_visual_check_v1/visual_review.json'
visual=dict(state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED',
    inspected_image='actual_marker_pairs.jpg',samples=16,observation='入口・旋回・出口の16抽出画像で実前緑と後赤/桃色マーカーへの重なりを確認。',
    whole_video_human_review_performed=False,acceptance_thresholds_unchanged=True)
if visual_path.exists():assert read(visual_path)==visual
else:save(visual_path,visual)
rows=[];settings=[];conditions=[];geometries=[]
for actual,analysis in pairs:
    geometry=read(analysis/'midpoint_raw_wheel_clock_v1/midpoint_review.json');geometries.append(geometry)
    assert geometry['independent_optical_geometry_eligible'] and geometry['clock']['passed']
    report=read(analysis/'full/report.json');trial=read(actual/'trial.json')
    identifier=actual.name.removeprefix('run_')
    operator=read(actual.parent/'operator_results'/(identifier+'.json'))
    job=read(actual.parent/'jobs'/(identifier+'.json'))
    assert operator['both_powered_off'] and operator['contact_detected'] is False
    assert job['request']['config']['address']=='E1:7F:86:21:84:73'
    assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
    actual_settings=read(actual/'saved_1ms.settings.json')
    settings.append({key:actual_settings[key] for key in ('mouse','motion_name','preset_speed_mm_s','parameters','pid','feedforward','suction','turn_count')})
    runtime=read(actual.parent.parent/'runtime_conditions_for_review.json')
    conditions.append({key:value for key,value in runtime.items() if key!='sources'})
    tracking=read(actual/'tracking_1ms_review_v3/verification.json')
    target=geometry['target_midpoint_Lend_right_forward_mm'];point=geometry['midpoint_Lend_right_forward_mm']
    stop=geometry['late_stationary']['midpoint_right_forward_mm'];stop_target=[112.5*math.sqrt(2),67.5*math.sqrt(2)]
    angle=geometry['exit_path_direction_change_deg']
    rows.append(dict(request_id=identifier,actual_trial=str(actual/'trial.json'),analysis_trial=str(analysis/'trial.json'),
        geometry_review=str(analysis/'midpoint_raw_wheel_clock_v1/midpoint_review.json'),
        geometry_review_sha256=sha(analysis/'midpoint_raw_wheel_clock_v1/midpoint_review.json'),
        exit_angle_deg=angle,Lend_right_forward_mm=point,Lend_error_mm=math.dist(point,target),
        Lstart_delta_from_exit_line_mm=geometry['Lstart_delta_from_exit_line_mm'],
        exit_line_transverse_offset_mm=geometry['Lstart_delta_from_exit_line_mm']*math.sin(math.radians(angle)),
        stop_right_forward_mm=stop,stop_error_mm=math.dist(stop,stop_target),
        Lend_to_stop_mm=math.dist(point,stop),
        required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
        registration_fraction=report['required_motion_window_quality']['calibration_rate'],
        tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],saturation_pct=tracking['tracking']['saturation_pct'],
        battery_min_v=tracking['battery_min_v'],clock_fit_rms_mm=geometry['clock']['incoming_raw_wheel_fit_rms_mm'],
        clock_half_spread_ms=geometry['clock']['half_window_zero_spread_ms'],operator_result=operator,
        capture_stop_evidence=trial['recording_stop']['log_stop_evidence']))
assert settings[0]==settings[1]
assert read(pairs[0][1]/'body_geometry_used.json')==read(current/'body_geometry_used.json')
assert conditions[0]==conditions[1], 'モード/前後走の実機条件が異なるため同一条件としない'
next_stage=base/'repeat_c1p0_Ls5p3_Le31_01'
next_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_repeat_green_20261121_001'
assert not list((next_stage/'local_execution/jobs').glob('*.json'))
candidate=read(next_stage/'candidate.settings.json')
assert candidate['parameters']==settings[0]['parameters']
assert read(next_stage/'config.json')['address']=='E1:7F:86:21:84:73'
assert read(Path(read(next_stage/'config.json')['geometry']))==read(current/'body_geometry_used.json')
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,
    successful_same_candidate_runs=2,settings=settings[0],same_runtime_conditions=conditions[0],runs=rows,
    mean_exit_angle_deg=statistics.mean(row['exit_angle_deg'] for row in rows),
    exit_angle_range_deg=max(row['exit_angle_deg'] for row in rows)-min(row['exit_angle_deg'] for row in rows),
    mean_Lend_right_forward_mm=[statistics.mean(row['Lend_right_forward_mm'][axis] for row in rows) for axis in (0,1)],
    maximum_Lend_error_mm=max(row['Lend_error_mm'] for row in rows),
    reference='実測入口基準の高さ補正マーカー中点。固定盤面/絶対車軸/機軸ではない。',
    original_first_failure_preserved=True,first_run_reanalysis_does_not_count_as_new_run=True,
    BLE_failed_request002_not_counted_as_run=True,power_state='BOTH_OFF_USER_CONFIRMED',
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    next_policy='走行設定と検出条件を維持し、同一候補3本目で平均/最大ずれ/ばらつきを確認。',
    adopted=False,source_params_changed=False,firmware_write_performed=False,
    limitations=['2本のみ。BのEKF係数、絶対車軸/機軸、独立ハード同期、連続接続は未確認。',
        '停止点は出口形状と別に評価し、約8mmの入口相対ずれを出口長へ一律に埋め込まない。',
        '公式full/summaryの判定は維持。停止確認は取得側連続ログ・動画静止・操作者報告を別証拠とする。'])
out.mkdir();save(out/'review.json',result)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6,7),layout='constrained')
for number,((actual,analysis),row) in enumerate(zip(pairs,rows),1):
    pose=pd.read_csv(analysis/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    valid=pose.registered_midpoint_valid.astype(bool)
    ax.plot(pose.right_mm.where(valid),pose.forward_mm.where(valid),label=f'{number}本目 {row["exit_angle_deg"]:.3f}°')
    ax.scatter(*row['Lend_right_forward_mm']);ax.scatter(*row['stop_right_forward_mm'],marker='+')
ax.scatter(*geometries[0]['target_midpoint_Lend_right_forward_mm'],marker='x',color='black',label='Lend目標')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='機体B1600 右135度出口 / 5.3・31.0 mm 補正+1.0°')
ax.grid(alpha=.3);ax.legend(fontsize=8);fig.savefig(out/'two_trajectories.png',dpi=140);plt.close(fig)
text='# 機体B1600 右135度出口・同一候補2本\n\n半径−41 mm・Lstart5.3 mm・Lend31.0 mm・補正+1.0°、吸引650、B PID/FF、モード/前後走条件と赤色相165..179の検出を揃えた。未走行のBLE失敗要求002は本数に含めない。\n\n|試行|入口相対出口角 [deg]|Lend右/前 [mm]|Lend目標差 [mm]|停止点目標差 [mm]|\n|---|---:|---:|---:|---:|\n'
for number,row in enumerate(rows,1):
    text+=f'|{number}|{row["exit_angle_deg"]:.3f}|{row["Lend_right_forward_mm"][0]:.3f}/{row["Lend_right_forward_mm"][1]:.3f}|{row["Lend_error_mm"]:.3f}|{row["stop_error_mm"]:.3f}|\n'
text+=f'\n平均角{result["mean_exit_angle_deg"]:.3f}°、2本間の角度差{result["exit_angle_range_deg"]:.3f}°。観測率98.742%/95.597%、毎評価フレームのArUco登録100%、閾値95%を維持。\n\n停止点には約8mmの入口相対差が残る。出口形状・Lend時刻との違いを保持し、固定盤面/車軸精度・停止精度/連続接続の合格とはしない。同じ設定の3本目を準備し、今回の配置/ON/手離し確認まで未送信。採用・ソース変更・書込みは未実施。\n'
(out/'README.md').write_text(text,encoding='utf-8')
save(base/'geometry003_review.json',dict(current_run=rows[1],same_candidate_review=str(out/'review.json'),next_request_id=next_id,
    next_request_sent=False,operator_result=rows[1]['operator_result'],source_params_changed=False,firmware_write_performed=False))
phase_path=current.parent.parent/'phase.json';phase=read(phase_path)
phase.update(state='CAPTURED_RELATIVE_REVIEWED_BOTH_OFF',next_candidate_executed=True,execution_pending=False,
    successful_run_count=1,review=str(base/'geometry003_review.json'))
phase_path.write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
state.update(state='GEOMETRY5P3_31_SUCCESS02_REVIEWED_THIRD_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,execution_pending=False,ble_connected=False,
    current_placement_confirmed=False,current_hands_clear_confirmed=False,successful_same_candidate_runs=2,
    last_run=str(current),last_request_id=rows[1]['request_id'],last_request_result='CAPTURED_RELATIVE_REVIEWED',last_error=None,
    relative_exit_rotation_deg=rows[1]['exit_angle_deg'],position_error_mm=rows[1]['Lend_error_mm'],
    video_required_observation_fraction=rows[1]['required_observation_fraction'],video_registration_fraction=1.,
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    comparison_review=str(out/'review.json'),next_action='出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/両方ON/手離し確認後、競合を再確認し、B用BLEで同一候補3本目を新IDで1回測る。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write(f'\n要求003で同一候補の2本目を取得。両方OFF・接触なし。1ms280点/drop0、走行中欠落/CRC0、飽和0%、最低7.781V。全895中888実ペア、必要区間95.597%、ArUco100%、抽出16画像で実マーカー確認。入口相対出口角{rows[1]["exit_angle_deg"]:.3f}度、Lend目標差{rows[1]["Lend_error_mm"]:.3f}mm、停止点差{rows[1]["stop_error_mm"]:.3f}mm。2本平均角{result["mean_exit_angle_deg"]:.3f}度。同じ設定/モード/検出で3本目を準備、次の操作者確認前は未送信。詳細geometry_same_candidate_two_review_v1/README.md。\n')
print(json.dumps(dict(runs=rows,mean_exit_angle_deg=result['mean_exit_angle_deg'],next_request_id=next_id),ensure_ascii=False),flush=True)
