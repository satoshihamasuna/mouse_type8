"""実走済みB右V90の暫定値を機体専用テーブルへ反映。角度未達を採用記録へ明記する。"""
from pathlib import Path
from datetime import datetime,timezone
import difflib,hashlib,json,shutil
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
STAGE=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls6p0_Le17p4_cm1p2_02'
RUN=next((STAGE/'local_execution').glob('run_*'))
OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_RV90_adoption'
BACKUP=ROOT/'tmp/B_RV90_source_before_provisional_20261121'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

m=read(RUN/'midpoint_raw_wheel_clock_v2/midpoint_review.json');settings=read(RUN/'saved_1ms.settings.json')
operator=read(STAGE/'operator_result.json');trial=read(RUN/'trial.json');track=read(RUN/'tracking_1ms_review_v3/verification.json')
assert m['independent_optical_geometry_eligible'] and operator['both_powered_off'] and not operator['contact_detected']
assert trial['mouse']=='B' and trial['motion']=='r_v90' and trial['preset']==1600 and trial['execution_resend_count']==0
assert settings['parameters']==dict(velo=1.6,r_min=-40.,lstart=6.,lend=17.4,degree=-90.,correction=-1.2,pre_accel=6.5)
assert settings['feedforward']['ff_om_accel']==.0008 and settings['feedforward']['ff_om_decel']==.0007
norm=float(np.linalg.norm(m['midpoint_Lend_residual_mm']));assert norm<=3.
goal_passed=bool(norm<=3 and abs(m['exit_path_error_from90_deg'])<=.5)
assert not OUT.exists() and not BACKUP.exists()
cpp=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp';header=cpp.with_suffix('.h')
original={p:p.read_bytes() for p in (cpp,header)};updated={}
for p,old in original.items():
    nl=b'\r\n' if b'\r\n' in old else b'\n';new=old
    def replace_once(before,after):
        global new
        assert new.count(before)==1,(str(p),before)
        new=new.replace(before,after,1)
    assert b'ff_gain_turnRV90_1600' not in old
    if p==cpp:
        comment='''// 2026-11-21 B右V90の実走済み暫定値。1600 mm/s・吸引duty650、同設定の適格実走1本。
// Lstart/Lend = 6.0/17.4 mm、右補正-1.20度、右専用FF加速0.000800/減速0.000700。
// 入口相対のLend誤差0.32mm、出口92.03度（角度目標未達）。再現性・連続接続は未確認。
// 根拠・残差・未確認事項: tools/turn_analysis/20261121_B_turn1600_RV90_adoption/README.md
'''.encode('utf-8').replace(b'\n',nl)
        anchor=b'//k = 250'+nl+b'const t_pid_gain sp_gain_turnV90_1600'
        replace_once(anchor,comment+anchor)
        gain=b'const t_pid_gain om_gain_turnV90_1600 = {0.20f,0.01f,0.0f};'
        block='''
const t_ff_gain ff_gain_turnRV90_1600 = {
	0.661202f,0.052929f,0.0f,
	0.009256f,0.000800f,0.000700f,0.0f,
	0.0f,0.0f,0.0f
};'''.encode('utf-8').replace(b'\n',nl)
        replace_once(gain,gain+block)
        table=b'const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,2.20f,20.83f,-90.0f,Turn_R, 1.60f};'
        replace_once(table,b'// '+table+nl+b'const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,6.0f,17.4f,-90.0f,Turn_R,-1.20f};')
        param=b'const t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};'
        replace_once(param,b'const t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_turnRV90_1600};')
    else:
        anchor=b'extern const t_pid_gain om_gain_turnV90_1600;'
        replace_once(anchor,anchor+nl+b'extern const t_ff_gain ff_gain_turnRV90_1600;')
    assert new.count(b'\r\n')==new.count(b'\n') if nl==b'\r\n' else b'\r\n' not in new
    updated[p]=new

OUT.mkdir();BACKUP.mkdir();changes={};patch=[]
for p,old in original.items():
    rel=p.relative_to(ROOT);dest=BACKUP/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(old)
    assert p.read_bytes()==old,'Source changed concurrently: '+str(p)
    p.write_bytes(updated[p])
    changes[str(rel)]=dict(before_sha256=hashlib.sha256(old).hexdigest(),after_sha256=sha(p),backup=str(dest))
    patch.extend(difflib.unified_diff(old.decode('utf-8-sig').splitlines(True),updated[p].decode('utf-8-sig').splitlines(True),fromfile='before/'+rel.as_posix(),tofile='after/'+rel.as_posix()))
(OUT/'source_patch.diff').write_text(''.join(patch),encoding='utf-8',newline='\n')
save(OUT/'source_changes.json',changes)
shutil.copyfile(STAGE/'candidate.settings.json',OUT/'candidate.settings.json')
shutil.copyfile(RUN/'midpoint_raw_wheel_clock_v2/midpoint_review.json',OUT/'midpoint_review.json')
shutil.copyfile(RUN/'tracking_1ms_review_v3/verification.json',OUT/'tracking_review.json')
shutil.copyfile(BASE/'geom6p0_17p4_correction_bracket.json',OUT/'correction_bracket.json')
points=pd.read_csv(RUN/'full/relative_marker_midpoint.csv');xy=points[['center_dx_mm','center_dy_mm']].to_numpy();directions={}
for name in ('entry','exit'):
    lo,hi=m[f'{name}_window_pts_s'];mask=points.detected&(points.time_video_s>=lo)&(points.time_video_s<=hi)
    a=xy[mask];d=np.linalg.svd(a-a.mean(0),full_matrices=False)[2][0]
    if d@(a[-1]-a[0])<0:d=-d
    directions[name+'_board_direction_deg']=float(np.degrees(np.arctan2(d[1],d[0]))%360)
evidence=dict(recorded_at=datetime.now(timezone.utc).isoformat(),status='PROVISIONAL_SOURCE_REFLECTION_ANGLE_GOAL_NOT_MET',
    mouse='B',motion='r_v90',preset_mm_s=1600,source_trial=str(RUN/'trial.json'),candidate=str(OUT/'candidate.settings.json'),
    source_reflected=True,firmware_write_performed=False,independent_same_setting_valid_runs=1,
    independent_optical_geometry_quality_passed=True,relative_geometry_goal_passed=goal_passed,
    entry_relative_exit_deg=m['exit_path_direction_change_deg'],exit_error_deg=m['exit_path_error_from90_deg'],
    Lend_residual_mm=m['midpoint_Lend_residual_mm'],Lend_residual_norm_mm=norm,**directions,
    automatic_precise_adoption_allowed=m['precise_parameter_adoption_allowed'],manual_provisional_reflection=True,
    decision='実走済み候補として暫定反映。Lend位置0.32mmは目標内だが出口92.03度は目標未達で、確定した調整値とは扱わない。同一値の再測定を継続する。',
    stream_period_ms=10,saved_log_period_ms=1,measured_saved_period_ms=track['measured_log_period_ms'],
    saved_stream_match_count=track['matched_stream_rows'],operator_contact_detected=False,operator_both_powered_off=True,
    board_fixed_connection_passed=False,absolute_axle_or_body_yaw_calibrated=False,EKF_B_calibrated=False,
    left_v90_tested=False,continuous_connection_tested=False,post_turn_ringing_resolved=False,
    physical_quality_guards_unchanged=True,additional_3mm_1degree_setup_gate_applied=False,
    source_changes=changes,input_sha256={str(p):sha(p) for p in [RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'telemetry.csv',RUN/'saved_1ms.settings.json',RUN/'full/report.json',STAGE/'operator_result.json']},
    validation_state='PENDING')
save(OUT/'evidence.json',evidence)
save(RUN/'provisional_source_reflection.json',evidence)
shutil.copyfile(Path(__file__),RUN/'analysis_sources'/Path(__file__).name)
state=read(BASE/'resume_state.json');state.update(state='RV90_PROVISIONAL_SOURCE_REFLECTED_VALIDATION_PENDING_BOTH_OFF',
    source_params_changed=True,firmware_write_performed=False,power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,execution_pending=False,
    current_review=str(OUT/'evidence.json'),provisional_adoption_folder=str(OUT),same_candidate_valid_runs=1,same_candidate_quality_valid_runs=1,
    same_candidate_goal_passed_runs=int(goal_passed),offline_analysis_sessions=[],
    next_action='B向けビルドとパラメータ共有リンク検証。暫定値は角度未達のため、同一値を再測定して再現性を確認。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(source_reflected=True,status=evidence['status'],exit_deg=evidence['entry_relative_exit_deg'],Lend_norm_mm=norm,folder=str(OUT),source_changes=changes),ensure_ascii=False))
