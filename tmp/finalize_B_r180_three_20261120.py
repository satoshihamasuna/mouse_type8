"""B1600右180度の同一候補3本を検証し、採用前の単体候補を保存。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timedelta,timezone
import copy
import csv
import hashlib
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
OUT=BASE/'final_single_turn_candidate'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
review_path=BASE/'r47p5_three_review_v1/video_comparison.json'
review=read(review_path)
assert review['repeat_count']==3 and review['all_relative_reviews_eligible']
assert review['identical_device_settings_verified'] and review['identical_camera_and_marker_detection_verified']
assert not OUT.exists()
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import archive_for_trial,verify_archive
keys=('schema_version','mouse','mouse_identity_source','motion_name','preset_speed_mm_s',
      'parameters','pid','feedforward','suction','turn_count')
rows=[];archives=[];fingerprints=[];sources=[];command_count=0
for item in review['comparisons']:
    run=Path(item['source_trial']).parent
    trial=read(run/'trial.json');settings=read(run/'saved_1ms.settings.json')
    tracking=read(run/'tracking_1ms_review_v2/verification.json')
    assert trial['execution_write_attempted'] is True and trial['execution_resend_count']==0
    assert settings['mouse']=='B' and settings['motion_name']=='long_r180' and settings['turn_count']==1
    assert settings['preset_speed_mm_s']==1600 and settings['suction']==dict(enabled=True,duty=650)
    assert tracking['cnt_contiguous'] and tracking['saved_log_status']['drop_count']==0
    assert tracking['tracking']['saturation_pct']==0 and tracking['battery_min_v']>=7
    operator=read(run.parent/'operator_results'/(run.name.removeprefix('run_')+'.json'))
    assert operator['contact_detected'] is False
    full=read(run/'full/report.json')
    assert full['required_motion_window_quality']['accepted_for_relative_review']
    assert full.get('detector',{}).get('inferred_points_used_as_observations',False) is False
    archive=archive_for_trial(run/'trial.json');assert archive is not None
    manifest=verify_archive(archive)
    archives.append(dict(folder=str(archive),verified=True,file_count=len(manifest['files'])))
    fingerprints.append(json.dumps({k:settings[k] for k in keys},sort_keys=True))
    rows.append(dict(sample=item['label'],rotation_deg=item['entry_to_exit_path_rotation_deg'],
        exit_spacing_mm=item['exit_line_right_at_forward45_mm'],forward_apex_mm=item['forward_apex_mm'],
        entry_board_heading_deg=item['entry_board_heading_deg'],exit_board_heading_deg=item['exit_board_heading_deg'],
        motion_observation_rate=item['motion_observation_rate'],motion_registration_rate=item['motion_calibration_rate'],
        maximum_valid_gap_ms=item['maximum_valid_motion_observation_gap_ms'],
        late_stationary_rms_mm=item['stationary_scatter_rms_mm'],
        tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],
        battery_min_v=tracking['battery_min_v'],measured_period_ms=tracking['measured_log_period_ms'],
        saved_1ms_count=tracking['sample_count'],archive_folder=str(archive),source_trial=str(run/'trial.json')))
    sources.extend([run/'trial.json',run/'saved_1ms.csv',run/'saved_1ms.settings.json',
        run/'telemetry.csv',run/'full/report.json',run/'full/apparent_floor_marker_pose.provenance.json',
        run.parent/'runtime_modes.json',run.parents[1]/'runtime_conditions_for_review.json'])
assert len(set(fingerprints))==1
last_operator=read(Path(review['comparisons'][-1]['source_trial']).parent.parent/'operator_results'/
    'B_R180_1600_r47p5_c0p5_s35_repeat_20261120_002.json')
assert last_operator['both_powered_off'] is True
candidate={key:copy.deepcopy(settings[key]) for key in keys}
candidate['source']='機体B・1600右180度・吸引650の同一RAM設定3本を全フレーム動画の実測入口基準で確認した単体候補。ファーム未反映。'
OUT.mkdir()


def save(name,value):
    with (OUT/name).open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)
        output.write('\n')


save('B_R180_1600_single_turn_candidate.settings.json',candidate)
stats={}
for key in ('rotation_deg','exit_spacing_mm','forward_apex_mm','tracking_rms_rad_s','battery_min_v'):
    v=np.array([row[key] for row in rows])
    stats[key]=dict(mean=float(v.mean()),minimum=float(v.min()),maximum=float(v.max()),
        range=float(np.ptp(v)),sample_std=float(v.std(ddof=1)))
original_hashes=read(BASE/'preparation.json')['source_hashes']
source_unchanged={key:hashlib.sha256((ROOT/key).read_bytes()).hexdigest()==value
    for key,value in original_hashes.items() if key.startswith(('Params','System','Task'))}
save('candidate.json',dict(status='PROVISIONAL_SINGLE_TURN_CANDIDATE_3_VALID_RUNS',
    created_at=datetime.now(timezone(timedelta(hours=9))).isoformat(),mouse='B',motion='long_r180',
    speed_mm_s=1600,settings_file='B_R180_1600_single_turn_candidate.settings.json',
    identical_parameter_run_count=3,identical_camera_and_detection_verified=True,
    evaluation_frame='実測入口直線と走行前マーカー中点を基準とする相対形状。',
    rows=rows,statistics=stats,archives=archives,
    current_power_state='BOTH_OFF_USER_CONFIRMED',ble_disconnected=True,pending_execution_request=None,
    no_contact_all_three_user_confirmed=True,source_params_changed_by_this_task=False,
    firmware_sources_match_preparation=source_unchanged,firmware_write_performed=False,
    firmware_parameter_adoption_performed=False,continuous_connection_verified=False,
    left_turn_verified=False,B_turn_ekf_identified=False,
    limitations=['既定の停止直後要約の未合格は保持。撮影時の停止確認と後半の静止動画・操作者確認を別に照合した。',
        '絶対車軸位置・高さ補正済み機首方向・格子の絶対尺度・独立同期は未校正。',
        'Lendは元の27.38mmを維持。停止後の進みや同期の±5ms差から長さを逆算していない。',
        '反復中の実機最低電圧は変動する。A・左180度・別速度・吸引条件へ流用しない。'],
    source_review=str(review_path),input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    report_sha256=hashlib.sha256(review_path.read_bytes()).hexdigest()))
with (OUT/'measurements.csv').open('x',encoding='utf-8-sig',newline='') as output:
    writer=csv.DictWriter(output,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
lines=['# 機体B・1600右180度の単体候補','',
    '同一設定3本を、実測入口方向・走行前マーカー中点をそろえた相対動画で確認した。正式採用・ファーム反映・連続接続は未実施。','',
    '候補：半径47.5 mm、Lstart13.12 mm、Lend27.38 mm、角度補正＋0.5度、速度1600 mm/s、吸引650。BのPID/FFは維持。','',
    '| 試行 | 回転量 [deg] | 幅 [mm] | 頂点 [mm] | 最低電圧 [V] |',
    '| --- | ---: | ---: | ---: | ---: |']
for r in rows:lines.append(f"| {r['sample']} | {r['rotation_deg']:.2f} | {r['exit_spacing_mm']:.2f} | {r['forward_apex_mm']:.2f} | {r['battery_min_v']:.3f} |")
lines+=['',f"平均：角度{stats['rotation_deg']['mean']:.2f}度、幅{stats['exit_spacing_mm']['mean']:.2f} mm、頂点{stats['forward_apex_mm']['mean']:.2f} mm。",'',
    '幅の目標90 mm・頂点参照185 mmに対する残差を残す。ばらつき・尺度未校正を無視して細かい値へ追い込まない。', '',
    '3本とも操作者が穴・壁への接触なしを確認。最終的に機体とHY0020の両方OFF・BLE切断を確認した。保存物はSHA-256で照合済み。','',
    '[3本の相対軌跡](../r47p5_three_review_v1/video_path_comparison.png)、[候補と根拠](candidate.json)、[RAM設定](B_R180_1600_single_turn_candidate.settings.json)。','',
    '機体Bの右180度のみの候補。左・連続・迷路走行、BのターンEKF係数同定は未確認。ソースのParams/Param_B/turn_1600/turn_1600.cppへは反映していない。']
with (OUT/'README.md').open('x',encoding='utf-8') as output:output.write('\n'.join(lines)+'\n')
print(json.dumps(dict(output=str(OUT),statistics=stats,archives_verified=len(archives),
    power='BOTH_OFF',source_unchanged=source_unchanged),ensure_ascii=False),flush=True)
