"""B右V90の保存済み2本を固定した赤v4条件で比較し、次の比較条件を記録する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OUT=BASE/'rear_red_profile_v4_validation'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def direction(points):
    d=np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]
    if d@(points[-1]-points[0])<0:d=-d
    return float(np.degrees(np.arctan2(d[1],d[0]))%360)

rows=[]
for i in (2,3):
    stage=BASE/f'geometry_ff0p0006_Ls3p3_Le18p9_c0p0_{i:02d}'
    run=next((stage/'local_execution').glob('run_*'))
    review=read(run/'red_v4_midpoint_clock_v1/midpoint_review.json')
    static=read(OUT/f'preflight_run{i}.redv4.json')
    track=read(run/'tracking_1ms_review_v3/verification.json')
    old=pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    new=pd.read_csv(run/'red_v4_full/apparent_floor_marker_pose.csv')
    assert np.array_equal(old.frame,new.frame)
    valid=(old.marker_detected_raw&new.marker_detected_raw&old.calibration_valid&new.calibration_valid).to_numpy()
    def rear(p):return 2*p[['midpoint_image_x_px','midpoint_image_y_px']].to_numpy()-p[['front_image_x_px','front_image_y_px']].to_numpy()
    shift=np.linalg.norm(rear(new)-rear(old),axis=1)[valid]
    trajectory=pd.read_csv(run/'red_v4_full/relative_marker_midpoint.csv')
    xy=trajectory[['center_dx_mm','center_dy_mm']].to_numpy()
    angles={}
    for name in ('entry','exit'):
        lo,hi=review[f'{name}_window_pts_s']
        mask=trajectory.detected&(trajectory.time_video_s>=lo)&(trajectory.time_video_s<=hi)
        angles[name+'_board_direction_deg']=direction(xy[mask])
    assert review['independent_optical_geometry_eligible'] and static['passed']
    assert review['independent_clock_required_motion_quality']['observation_rate']==1
    residual=float(np.linalg.norm(review['midpoint_Lend_residual_mm']))
    rows.append(dict(run=str(run),review=str(run/'red_v4_midpoint_clock_v1/midpoint_review.json'),
        operator_result=str(stage/'operator_result.json'),quality_valid=True,
        entry_relative_exit_deg=review['exit_path_direction_change_deg'],Lend_residual_mm=review['midpoint_Lend_residual_mm'],
        Lend_residual_norm_mm=residual,geometry_goal_passed=bool(residual<=3 and abs(review['exit_path_error_from90_deg'])<=.5),
        **angles,static_midpoint_mm=static['center_board_mm'],static_apparent_heading_deg=static['heading_deg'],
        raw_motion_observation_rate=review['independent_clock_required_motion_quality']['observation_rate'],
        common_old_new_pair_frames=int(valid.sum()),rear_centroid_shift_px_quantiles=np.quantile(shift,[0,.5,.95,.99,1]).tolist(),
        gyro_through_Lend_deg=track['gyro_through_lend_deg'],turn_tracking_rms_rad_s=track['tracking']['tracking_rms_rad_s'],
        post_tracking_rms_rad_s=track['tracking']['post_rms_rad_s'],loaded_battery_min_v=track['battery_min_v']))

visual=read(OUT/'visual_validation.json')
profile=Path(visual['profile'])
assert hashlib.sha256(profile.read_bytes()).hexdigest()==visual['profile_sha256']
comparison=dict(created_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='r_v90',preset_mm_s=1600,
    motor_settings_identical=True,geometry_profile=str(profile),profile_sha256=visual['profile_sha256'],trials=rows,
    exit_angle_spread_deg=abs(rows[0]['entry_relative_exit_deg']-rows[1]['entry_relative_exit_deg']),
    static_heading_difference_deg=abs(rows[0]['static_apparent_heading_deg']-rows[1]['static_apparent_heading_deg']),
    causal_conclusion='未確定。入口相対角は設置の回転だけでは変わらない。設置・進入・後走の差を切り分けるため開始姿勢を揃えて追加比較する。',
    control_limitation='旋回後に角速度の振れが残る。幾何の合格だけで制御追従や連続接続を完了扱いしない。',
    prospective_setup_gate=dict(representative_midpoint_error_max_mm=3.,representative_heading_error_max_deg=1.,
        target_midpoint_mm=[315.,90.],target_apparent_heading_deg=135.,
        original_static_quality_guards_unchanged=True,applies_to_future_runs_only=True),
    source_params_changed=False,hardware_operations=0)
save(OUT/'c0p0_two_run_comparison.json',comparison)
save(OUT/'full_validation.json',dict(created_at=comparison['created_at'],profile=str(profile),profile_sha256=visual['profile_sha256'],
    visual_validation=str(OUT/'visual_validation.json'),full_comparison=str(OUT/'c0p0_two_run_comparison.json'),
    full_videos_validated=2,static_videos_validated=2,required_motion_pair_observation_rate=[1.,1.],
    every_measurement_frame_registered=True,interpolated_pairs_used=False,original_failures_preserved=True,
    physical_quality_guards_unchanged=True,approved_for_live_capture=True,
    approval_scope='B前緑・後赤、現撮影設定の静止・走行実マーカー検出。各新規試行でも静止と全フレームの品質を再検査する。'))
state=read(BASE/'resume_state.json')
state.update(state='C0P0_REPRODUCIBILITY_REVIEW_COMPLETE_BOTH_OFF',same_candidate_quality_valid_runs=2,
    same_candidate_goal_passed_runs=sum(r['geometry_goal_passed'] for r in rows),same_candidate_valid_runs=2,
    previous_exit_angles_deg=[r['entry_relative_exit_deg'] for r in rows],
    current_parameters=read(BASE/'geometry_ff0p0006_Ls3p3_Le18p9_c0p0_03/candidate.settings.json')['parameters'],
    detector_full_validation=str(OUT/'full_validation.json'),
    next_action='赤v4と追加の代表開始姿勢条件を用意し、補正0.00°のまま1回比較。新しい操作者準備確認後だけ実行。',
    execution_pending=False,operator_ready=False,ble_connected=False,power_state='BOTH_OFF_USER_CONFIRMED')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(comparison,ensure_ascii=False,indent=2))
