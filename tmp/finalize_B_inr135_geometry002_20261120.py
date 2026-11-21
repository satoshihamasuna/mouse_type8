"""充電中のB1600右135度入口候補・次の反復条件を保存する。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
STAGE=BASE/'geometry_c0p4_Ls10p5_Le17p2_02'
SESSION=STAGE/'local_execution'
RID='B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002'
RUN=SESSION/('run_'+RID)
OUT=BASE/'geometry_candidate_review_v1'
assert not OUT.exists();OUT.mkdir()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False)


trial,actual,track,geometry,operator=map(read,[RUN/'trial.json',RUN/'saved_1ms.settings.json',
    RUN/'tracking_1ms_review_v3/verification.json',RUN/'midpoint_raw_wheel_clock_v1/midpoint_review.json',
    SESSION/'operator_results'/f'{RID}.json'])
assert geometry['independent_optical_geometry_eligible'] and operator['both_powered_off'] and not operator['contact_detected']
assert actual['parameters']==dict(velo=1.6,r_min=-42.5,lstart=10.5,lend=17.2,degree=-135.,correction=.4,pre_accel=6.5)
residual=np.array(geometry['midpoint_Lend_residual_mm'])
failed_stage=BASE/'geometry_c0p4_Ls10p5_Le17p2_01/local_execution'
failed_id='B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_001'
failed=read(failed_stage/'jobs'/f'{failed_id}.json')['result']
assert failed['execution_write_attempted'] is False and failed['ble_disconnected'] is True
save(OUT/'unexecuted_geometry001.json',dict(request_id=failed_id,state='NOT_EXECUTED_BLE_DISCONNECTED_BEFORE_DEVICE_CHECK',
    execution_write_attempted=False,settings_write_commands_sent=0,preset_selections_sent=0,stream_rows=803,
    stream_lost=0,stream_crc_errors=0,ideal_velo_and_rad_velo_all_zero=True,battery_range_v=[7.5,7.51171875],
    source_result=failed,source_request=str(failed_stage/'jobs'/f'{failed_id}.json'),
    power_cycle_and_position_source=str(BASE/'operator_ready_geometry002.json'),original_evidence_retained=True))
pre=SESSION/('camera_before_ble_'+RID)/'static_check.json'
transport=SESSION/('transport_'+RID)
sequence=dict(state='COMPLETED_ONE_RUN_WITH_CAMERA_BEFORE_BLE',static_result=read(pre),
    static_finished_file_mtime_ns=pre.stat().st_mtime_ns,transport_created_ns=transport.stat().st_ctime_ns,
    static_file_precedes_transport_creation=pre.stat().st_mtime_ns<=transport.stat().st_ctime_ns,
    result=read(SESSION/'jobs'/f'{RID}.json')['result'],hardware_disconnect_cause_proven=False,
    scope='今回の一試行で接続から保存まで完了。普遍的なBLE切断対策の実証とは区別。')
save(OUT/'camera_before_ble_observed_sequence.json',sequence)

rows=[]
sources=[('初回 +2.10° / 5.1・20.56mm',BASE/'baseline01/local_execution/run_B_INR135_1600_baseline_green_20261120_002/recovered_metadata_v1'),
    ('角度比較 −0.40° / 5.1・20.56mm',BASE/'angle_cm0p4_01/local_execution/run_B_INR135_1600_cm0p4_green_20261120_001'),
    ('新候補 +0.40° / 10.5・17.2mm',RUN)]
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(7,7.5),layout='constrained')
for label,run in sources:
    review=read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
    frame=pd.read_csv(run/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    quality=review['independent_clock_required_motion_quality']
    mask=(frame.time_video_s>=quality['video_motion_start_s']-.02)&(frame.time_video_s<=quality['video_motion_end_s']+.05)
    xy=np.where(frame.registered_midpoint_valid.to_numpy(bool)[:,None],frame[['right_mm','forward_mm']].to_numpy(),np.nan)
    curve,=ax.plot(*xy[mask].T,label=label)
    ax.scatter(*review['midpoint_Lend_right_forward_mm'],c=curve.get_color(),s=45)
    rows.append(dict(label=label,run=str(run),exit_rotation_deg=review['exit_path_direction_change_deg'],
        Lend_right_forward_mm=review['midpoint_Lend_right_forward_mm'],Lend_residual_mm=review['midpoint_Lend_residual_mm']))
ax.plot([90,180],[135,45],'--',color='gray',label='目標出口線')
ax.scatter(90,135,c='black',marker='x',s=60,label='Lend目標(90,135)mm')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',
    title=f'機体B1600 右135度入口：候補単発\n新候補 {geometry["exit_path_direction_change_deg"]:.2f}° / Lend差 {np.linalg.norm(residual):.2f}mm')
ax.grid(alpha=.3);ax.legend(fontsize=8);fig.savefig(OUT/'comparison.png',dpi=145);plt.close(fig)
next_stage=BASE/'geometry_c0p4_Ls10p5_Le17p2_repeat_01'
next_id=read(next_stage/'phase.json')['request_id']
assert read(next_stage/'candidate.settings.json')['parameters']==actual['parameters']
result=dict(state='SINGLE_RELATIVE_GEOMETRY_CANDIDATE_REVIEWED_REPEATS_PENDING',mouse='B',motion='in_r135',preset_mm_s=1600,
    settings=actual,source_trial=str(RUN/'trial.json'),source_review=str(RUN/'midpoint_raw_wheel_clock_v1/midpoint_review.json'),
    exit_rotation_deg=geometry['exit_path_direction_change_deg'],rotation_error_deg=geometry['exit_path_direction_change_deg']-135,
    Lend_right_forward_mm=geometry['midpoint_Lend_right_forward_mm'],Lend_residual_mm=residual.tolist(),Lend_residual_norm_mm=float(np.linalg.norm(residual)),
    tracking=track,all_position_evaluation_frames=813,observed_frames=813,registered_aruco_frames=813,
    required_motion_quality=geometry['independent_clock_required_motion_quality'],late_stationary=geometry['late_stationary'],
    original_summary=read(RUN/'full/summary.json'),recording_stop=trial['recording_stop'],operator_result=operator,
    same_candidate_valid_runs=1,independent_repeat_goal=3,remaining_same_candidate_repeats=2,
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,execution_pending=False,
    power_state='BOTH_OFF_USER_CONFIRMED',charging_in_progress=True,charging_completion_confirmed=False,
    resume=dict(charging_completion_required=True,placement_mm=[315.,135.],front_green_heading_deg=90.,
        current_clearance_and_on_hands_clear_confirmation_required=True,camera_health_before_on_required=True,
        readback_after_power_cycle=True,post_diagonal_accel_m_s2=20.,
        order=['充電完了・両方OFF確認','カメラ端末残量の再確認','元配置/穴/全接地点/車幅/旋回/停止余裕確認','両方ON・手離しの今回確認',
            '静止動画を転送・照合','BLEへ接続','B1600対象プリセットと全候補を復元・照合','新IDで1回のみ撮影走行']),
    source_params_changed=False,firmware_write_performed=False,firmware_build_required=False,
    firmware_source_hashes_unchanged={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==value
        for path,value in read(BASE/'preparation.json')['source_hashes'].items() if path.replace('\\','/').startswith(('Params/','System/','Task/'))},
    geometry_comparison=rows,archive=read(RUN/'archive.json'),
    turn_EKF_status='Bの幾何/校正と解析ツール対応が未完了。K・雑音は未同定/未反映。',
    right_exit_status=dict(state='PREPARED_NOT_EXECUTED',folder=str(ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'),
        placement_mm=[180.,45.],front_green_heading_deg=45.,current_clearance_not_confirmed=True),
    limitations=['相対中点の単発候補。独立3本の再現性・連続接続・左右対称性は未確認。',
        '盤面固定の開始姿勢/停止点と車軸の絶対校正の合格ではない。',
        '既定summaryのauto_stop未合格を保持。撮影制御の1349ms静止、後半動画の静止と操作者確認を別の根拠とする。'])
save(OUT/'review.json',result)
(OUT/'README.md').write_text('機体B1600右135度入口の単発相対候補。補正+0.40度、半径-42.5mm、Lstart10.5mm、Lend17.2mm、吸引650。\n\n'
    f'実測回転量{result["exit_rotation_deg"]:.4f}度、Lendは右{result["Lend_right_forward_mm"][0]:.3f}/前{result["Lend_right_forward_mm"][1]:.3f}mm。'
    f'目標90/135mmからの差は{result["Lend_residual_norm_mm"]:.3f}mm。813評価フレームで観測/実ArUco登録100%。\n\n'
    'geometry001は走行前にBLE切断し、設定変更/走行は0。002は静止撮影と転送をBLE接続前に終える順序で1回走行・保存。'
    '操作者が両方OFF・接触なしを確認。最低7.270Vのため充電を依頼し、ユーザーは「一度充電します」と回答。\n\n'
    '充電後は同じ設定の独立反復をあと2本確認してから右135度出口へ進む。元315/135mm・緑前下向きへ戻し、'
    '今回の配置/干渉なし/両方ON/手離しを確認するまで走行しない。新しい静止撮影、Bの実機値と後走20m/s²の読戻し、'
    '固有ID1回の順序を維持。既使用IDを再送しない。\n\n'
    'ファーム/Paramsは未変更。BのEKF係数は未同定。固定盤面/車軸絶対姿勢・独立反復3本・連続接続は未確認。\n',encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state=result['state'],power_state=result['power_state'],charging_requested=True,charging_in_progress=True,
    charging_completion_confirmed=False,next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    execution_pending=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    same_candidate_valid_runs=1,remaining_same_candidate_repeats=2,relative_exit_rotation_deg=result['exit_rotation_deg'],
    candidate_geometry_review=str(OUT/'review.json'),current_RAM_candidate_applied=False,
    next_action='充電完了・両方OFFを確認後、カメラ残量と元配置を照合。今回のON/干渉なし/手離し確認後に同じ候補をあと2本測定し、右135度出口へ進む。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as file:
    file.write('\n\n最新: geometry_candidate_review_v1/review.jsonとREADME.md。候補+0.4度/10.5/17.2mmは単発で134.379度・Lend差0.994mm。'
        '現在は充電中・両方OFF。独立反復あと2本の準備済み。右出口は未実走。\n')
print(json.dumps(dict(exit_rotation_deg=result['exit_rotation_deg'],Lend_residual_mm=result['Lend_residual_norm_mm'],
    same_candidate_valid_runs=1,power_state=result['power_state'],charging_in_progress=True,next_request_sent=False,
    firmware_source_hashes_unchanged=result['firmware_source_hashes_unchanged']),ensure_ascii=False))
