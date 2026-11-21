"""B右V90の同一条件4本を保持し、FFだけを変えた比較を準備する。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import shutil
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD=BASE/'geometry_ff0p0006_Ls3p3_Le18p9_c0p0_07'
NEW=BASE/'geometry_ff0p0008_Ls3p3_Le18p9_c0p0_01'
RID='B_RV90_1600_geom3p3_18p9_c0p0_ff0p0008_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def board_direction(p):
    d=np.linalg.svd(p-p.mean(0),full_matrices=False)[2][0]
    if d@(p[-1]-p[0])<0:d=-d
    return float(np.degrees(np.arctan2(d[1],d[0]))%360)

rows=[]
for i in (2,3,4,7):
    stage=BASE/f'geometry_ff0p0006_Ls3p3_Le18p9_c0p0_{i:02d}'
    run=next((stage/'local_execution').glob('run_*'))
    revised=i in (2,3)
    full=run/('red_v4_full' if revised else 'full')
    review_path=run/('red_v4_midpoint_clock_v1' if revised else 'midpoint_raw_wheel_clock_v2')/'midpoint_review.json'
    m=read(review_path);t=read(run/'tracking_1ms_review_v3/verification.json')
    assert m['independent_optical_geometry_eligible']
    profile=read(full/'body_geometry_used.json')
    assert profile['marker_detection']['red']['minimum_saturation']==30
    prep=next((stage/'local_execution').glob('prepare_*'))
    static=read(BASE/f'rear_red_profile_v4_validation/preflight_run{i}.redv4.json') if revised else read(prep/'static_check.json')
    points=pd.read_csv(full/'relative_marker_midpoint.csv')
    xy=points[['center_dx_mm','center_dy_mm']].to_numpy()
    angles={}
    for name in ('entry','exit'):
        lo,hi=m[f'{name}_window_pts_s']
        mask=points.detected&(points.time_video_s>=lo)&(points.time_video_s<=hi)
        angles[name+'_board_direction_deg']=board_direction(xy[mask])
    norm=float(np.linalg.norm(m['midpoint_Lend_residual_mm']))
    rows.append(dict(stage=i,trial=str(run/'trial.json'),geometry_review=str(review_path),
        quality_passed=True,entry_relative_exit_deg=m['exit_path_direction_change_deg'],Lend_residual_norm_mm=norm,
        Lend_residual_mm=m['midpoint_Lend_residual_mm'],
        relative_geometry_goal_passed=bool(norm<=3 and abs(m['exit_path_error_from90_deg'])<=.5),
        **angles,static_apparent_midpoint_board_mm=static['center_board_mm'],static_apparent_heading_deg=static['heading_deg'],
        board_fixed_exit_error_deg=angles['exit_board_direction_deg']-225.,
        absolute_axle_or_body_yaw_calibrated=False,
        turn_tracking_rms_rad_s=t['tracking']['tracking_rms_rad_s'],
        post_tracking_rms_rad_s=t['tracking']['post_rms_rad_s'],loaded_battery_min_v=t['battery_min_v']))

comparison=dict(created_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='r_v90',preset_mm_s=1600,
    source_settings=str(OLD/'candidate.settings.json'),settings_identical=True,detector_profile='rear_red_v4',
    trials=rows,all_valid_trials_retained=True,excluded_real_runs=0,
    failed_preflight_stages=[5,6],failed_preflight_execution_count=0,
    exit_mean_deg=float(np.mean([r['entry_relative_exit_deg'] for r in rows])),
    exit_range_deg=[min(r['entry_relative_exit_deg'] for r in rows),max(r['entry_relative_exit_deg'] for r in rows)],
    geometry_passed_runs=sum(r['relative_geometry_goal_passed'] for r in rows),
    total_quality_valid_runs=4,geometry_candidate_adopted=False,source_params_changed=False,
    reason='入口相対位置は4本とも3mm内だが出口角度は2/4本が90±0.5度の外。旋回後の角速度の振れも未解決。')
save(BASE/'c0p0_ff0p0006_four_run_review.json',comparison)
run=next((OLD/'local_execution').glob('run_*'))
save(run/'relative_geometry_review.json',dict(comparison=str(BASE/'c0p0_ff0p0006_four_run_review.json'),**rows[-1],
    original_summary_preserved=True,operator_approximate_placement_requested=True,
    additional_3mm_1degree_gate_applied=False,geometry_goal_tolerances_unchanged=True,
    board_fixed_connection_passed=False,source_params_changed=False))

assert not NEW.exists();NEW.mkdir()
for name in ('baseline.settings.json','config.json','static_pose_after_settle.py','static_5s_revision.json'):
    shutil.copyfile(OLD/name,NEW/name)
candidate=read(OLD/'candidate.settings.json')
assert candidate['feedforward']['ff_om_accel']==candidate['feedforward']['ff_om_decel']==.0006
candidate['feedforward'].update(ff_om_accel=.0008,ff_om_decel=.0008)
candidate['source']='B右V90の4本で旋回加減速時の残留PIDと末尾の振れを確認。閉ループの暫定FF推定は加速0.00112〜0.00116・減速0.00101〜0.00105だが励起不足警告あり。長さ/補正/PIDを維持し、FFだけを小幅の0.000800で比較。'
save(NEW/'candidate.settings.json',candidate)
old_id=read(OLD/'comparison_plan.json')['request_id']
wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count(old_id)==1
wrapper=wrapper.replace(old_id,RID)
compile(wrapper,str(NEW/'run_recorded_once.py'),'exec')
(NEW/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
d=pd.read_csv(run/'saved_1ms.csv')
acc=np.gradient(d['ideal.rad_velo'].to_numpy(),.001)
util=np.maximum(abs(d.V_r+.0002*acc),abs(d.V_l+.0002*acc))/d.Battery
plan=read(OLD/'comparison_plan.json')
plan.update(request_id=RID,independent_attempt=1,parameters=candidate['parameters'],pid=candidate['pid'],feedforward=candidate['feedforward'],
    changes=dict(ff_om_accel=[.0006,.0008],ff_om_decel=[.0006,.0008]),
    source_trial=str(run/'trial.json'),source_comparison=str(BASE/'c0p0_ff0p0006_four_run_review.json'),
    source_reviews=[r['geometry_review'] for r in rows],
    reason=candidate['source'],settings_identical_to=None,power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,
    execution_commands_sent=0,hardware_operations=0,prospective_setup_gate_applied=False,
    placement_protocol='ユーザー指定の概略設置。従来10mm/5度と静止安定度を維持し、入口相対軌道と盤面固定差を分ける。',
    ff_estimate_limitations=['B単方向・同一速度/吸引条件の閉ループ近似。励起不足のため独立同定ではない。',
        '並進FFと角速度比例FFは今回変更しない。推定値をそのまま採用しない。'],
    voltage_screen=dict(method='saved voltages plus delta FF * central derivative of ideal omega; feedback held fixed',
        maximum_battery_utilization=float(util.max()),not_closed_loop_prediction=True),
    acceptance='角速度追従RMS・加減速の残留PID・末尾RMS/振れ・電圧飽和を前回と比較。元の動画品質基準とLend3mm/出口90±0.5度を維持し、FF変更後の幾何を再測定。',
    source_params_changed=False,firmware_write_performed=False)
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
state=read(BASE/'resume_state.json')
state.update(state='FF0P0008_CANDIDATE_PREPARED_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',
    same_candidate_quality_valid_runs=4,same_candidate_goal_passed_runs=2,same_candidate_valid_runs=4,
    current_review=str(run/'relative_geometry_review.json'),four_run_comparison=str(BASE/'c0p0_ff0p0006_four_run_review.json'),
    next_action='ユーザーの新規準備返答後、元の静止品質検査を通過した場合だけFF0.000800を1回測定。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(folder=str(NEW),request_id=RID,previous_exit_deg=rows[-1]['entry_relative_exit_deg'],
    previous_Lend_norm_mm=rows[-1]['Lend_residual_norm_mm'],old_candidate_mean_exit_deg=comparison['exit_mean_deg'],
    changed_ff=[.0006,.0008],hardware_operations=0),ensure_ascii=False))
