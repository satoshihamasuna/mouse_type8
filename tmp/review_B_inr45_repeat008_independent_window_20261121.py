"""録画全体の失敗を保持し、PTS連続な測定区間だけを既存関数で独立評価する。"""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
REL='geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008'
RUN=BASE/REL
OUT=RUN/'independent_motion_window_v1'
OUT.mkdir(exist_ok=False)
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
import pixel_camera_relative_pose as core
from pixel_camera_tuning_report import summarize
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
def save(p,v):
    with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
trial=read(RUN/'trial.json');capture_path=Path(trial['capture_json']);capture=read(capture_path)
original_hashes={str(p):core.file_hash(p) for p in (RUN/'trial.json',capture_path)}
inspection=read(RUN/'pts_gap_review_v1/inspection.json')
pts=np.asarray(read(RUN/'pts_gap_review_v1/decoded_pts.json'),dtype=float)
assert trial['state']==trial['camera_state']==capture['state']=='FAILED'
assert trial['mouse']=='B' and trial['motion']=='in_r45' and trial['preset']==1600
assert trial['camera_run_start_confirmed'] and trial['execution_write_attempted']
assert trial['execution_resend_count']==0 and trial['log_quality']['lost_frames']==0 and trial['log_quality']['crc_errors_during_trial']==0
assert capture.get('error') is None and capture['video_inspection']['errors']==['DECODE_OR_PTS_ERROR']
assert inspection['hash_and_size_rechecked'] and inspection['original_failure_preserved']
assert core.file_hash(trial['video_file'])==capture['sha256']==inspection['video_sha256']
assert all(inspection[key]==0 for key in ('corrupt_frames','missing_pts','non_increasing_pts','wrong_size_frames'))
assert len(pts)==inspection['decoded_frames'] and np.isfinite(pts).all() and (np.diff(pts)>0).all()
start=max(0.,trial['video_motion_start_estimate_s']-.35)
end=trial['recording_stop']['confirmation_camera_seconds']+.65
intervals=np.diff(pts);touching=(pts[:-1]<=end)&(pts[1:]>=start)
selected=pts[(pts>=start)&(pts<=end)]
assert pts[0]<=start<end<=pts[-1] and len(selected)>200
assert np.max(intervals[touching])<=1.5/240+1e-12
fps=(len(selected)-1)/(selected[-1]-selected[0]);assert 239<=fps<=241
assert trial['recording_stop']['log_stop_evidence']['confirmed']
assert trial['recording_stop']['postroll_camera_seconds']>=1.
assert all(gap['before_pts_s']>trial['recording_stop']['decision_camera_seconds']+.15 for gap in inspection['gaps'])
qualification=dict(scope='INDEPENDENT_CONTIGUOUS_MOTION_WINDOW_ONLY',
    global_capture_passed=False,original_trial_state=trial['state'],original_capture_state=capture['state'],
    original_failure_preserved=True,selected_window_timing_valid=True,window_s=[start,end],
    selected_frames=len(selected),selected_fps_from_pts=float(fps),max_interval_touching_window_ms=float(np.max(intervals[touching])*1000),
    whole_video_gaps=inspection['gaps'],all_gaps_after_confirmed_postroll=True,
    original_input_sha256=original_hashes,hardware_commands_sent=0,parameter_adoption_allowed=False)
save(OUT/'window_qualification.json',qualification)
geometry=core.load_detection_geometry(RUN/'body_geometry_used.json')
reference=read(RUN/'marker_reference_used.json')
settings=read(RUN/'motion.settings.json')
assert settings['mouse']==geometry['mouse']==trial['mouse']
assert geometry.get('axle_offset_from_marker_midpoint_mm') is None
pose,_,_,_=core.read_video_pose(Path(trial['video_file']),geometry['front_colour'],marker_reference=reference,
    video_window=(start,end),marker_detection=geometry.get('marker_detection'),rear_colour=geometry['rear_colour'],
    nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),marker_span_limits_mm=geometry.get('marker_span_limits_mm'),
    frame_stride=1,progress=lambda phase,count,stamp:print(f'{phase}: {count} frames, PTS {stamp:.3f} s',flush=True))
pose.to_csv(OUT/'apparent_floor_marker_pose.csv',index=False)
provenance=dict(schema_version=1,video_file=trial['video_file'],video_sha256=core.file_hash(trial['video_file']),
    cached_pose_sha256=core.file_hash(OUT/'apparent_floor_marker_pose.csv'),reference_digest=core.reference_digest(reference),
    front_colour=geometry['front_colour'],rear_colour=geometry['rear_colour'],nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
    marker_span_limits_mm=geometry.get('marker_span_limits_mm'),marker_detection=geometry.get('marker_detection'),
    video_window=[start,end],frame_stride=1,marker_calibration=pose.attrs['marker_calibration'],
    detector_pipeline_version=2,body_detection=pose.attrs.get('body_detection',{}),
    global_capture_passed=False,qualification_scope=qualification['scope'])
save(OUT/'apparent_floor_marker_pose.provenance.json',provenance)
transform=np.linalg.inv(core.board_to_pose_transform(reference))
front=core.transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),transform)
rear=core.transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),transform)
relative=core.relative_marker_midpoint(front,rear,trial['camera_height_mm'],geometry['front_height_mm'],geometry['rear_height_mm'],
    pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool))
xy,valid=relative['displacement_xy_mm'],relative['valid'];time=pose.time_video_s.to_numpy()
baseline=valid&(time<=start+.2);assert baseline.sum()>=10
xy-=np.nanmedian(xy[baseline],axis=0)
pd.DataFrame(dict(frame=pose.frame,time_video_s=time,detected=valid,center_dx_mm=xy[:,0],center_dy_mm=xy[:,1])).to_csv(OUT/'relative_marker_midpoint.csv',index=False)
assert len(time)==len(selected) and np.max(np.abs(time-selected))<.001
calibration_valid=pose.calibration_valid.to_numpy(bool)
quality=dict(body_detection_rate=float(valid.mean()),frames=len(valid),valid_frames=int(valid.sum()),baseline_observations=int(baseline.sum()),
    marker_calibration=pose.attrs['marker_calibration'],detector=pose.attrs.get('body_detection',{}))
alignment=pose.copy();board_to_legacy=core.board_to_pose_transform(reference)[:2,:2]
aligned_xy=xy@board_to_legacy.T
alignment['center_x_mm']=pd.Series(aligned_xy[:,0]).interpolate(limit_direction='both')
alignment['center_y_mm']=pd.Series(aligned_xy[:,1]).interpolate(limit_direction='both')
filled=alignment[['center_x_mm','center_y_mm']].to_numpy()
alignment['speed_playback_mm_s']=np.linalg.norm(np.gradient(filled,time,axis=0),axis=1)
rawlog=pd.read_csv(trial['type8_log_csv']);origin_ms=float(rawlog.time_ms.iloc[0])
required_interval=(((trial['log_quality']['motion_start_time_ms']-origin_ms)%2**32)/1000-.2,
    ((trial['log_quality']['motion_end_time_ms']-origin_ms)%2**32)/1000+.1)
comparison,_,offset,_,_,rms=core.fit_time_and_pose(alignment,core.integrate_log(Path(trial['type8_log_csv'])),1.0,required_interval)
for columns in (['center_x_mm','center_y_mm'],['log_x_mm','log_y_mm']):
    comparison[columns]=comparison[columns].to_numpy()@np.linalg.inv(board_to_legacy).T
comparison.rename(columns={'heading_rad':'apparent_marker_angle_rad_not_height_corrected'},inplace=True)
comparison.to_csv(OUT/'relative_video_log_comparison.csv',index=False)
motion_quality=core.required_motion_quality(time,valid,calibration_valid,rawlog.time_ms.to_numpy(),trial,offset,1)
assert motion_quality['accepted_for_relative_review']
report=dict(status='INDEPENDENT_WINDOW_REVIEW_REQUIRED',mouse='B',motion='in_r45',frame_stride=1,observation_scope='all_frames_in_qualified_window',
    trial_json=str(RUN/'trial.json'),video_file=trial['video_file'],csv=trial['type8_log_csv'],settings_json=trial['type8_settings_json'],
    camera_height_mm=trial['camera_height_mm'],height_source='approximate_for_capture',quality=quality,
    absolute_position_available=False,height_corrected_heading_available=False,parameter_adoption_allowed=False,
    global_capture_passed=False,original_failure_preserved=True,window_qualification=str(OUT/'window_qualification.json'),
    trajectory_fit_rmse_mm=float(rms),log_time_zero_at_video_s=float(offset),required_motion_window_quality=motion_quality,
    input_sha256={str(p):core.file_hash(p) for p in (RUN/'trial.json',capture_path,RUN/'motion.csv',RUN/'motion.settings.json',
        RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',Path(__file__))})
save(OUT/'report.json',report)
try:summary=summarize(OUT)
except Exception as error:summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
save(OUT/'summary.json',summary)

# 入口raw wheel時計・直線品質・20ms補間上限は従来と同じ処理を使う。
original=ROOT/'tmp/review_B_inr45_midpoint_20261120.py'
source=original.read_text(encoding='utf-8')
source=source.replace("FULL=RUN/'full'","FULL=RUN/'independent_motion_window_v1'")
source=source.replace("OUT=RUN/'midpoint_raw_wheel_clock_v3'","OUT=RUN/'midpoint_independent_window_v1'")
source=source.replace('nominal_half_span_used=False,absolute_axle_or_body_yaw_available=False,',
    "nominal_half_span_used=False,absolute_axle_or_body_yaw_available=False,global_capture_passed=False,original_failure_preserved=True,qualification_scope='independent_contiguous_motion_window_only',")
driver=OUT/'midpoint_analysis_source.py';driver.write_text(source,encoding='utf-8')
with (OUT/'midpoint.stdout.txt').open('x',encoding='utf-8') as log:
    subprocess.run([sys.executable,str(driver),REL],check=True,stdout=log,stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0,env=dict(os.environ,PYTHONUTF8='1'),timeout=60)
geometry_result=read(RUN/'midpoint_independent_window_v1/midpoint_review.json')
assert geometry_result['independent_optical_geometry_eligible']
assert all(core.file_hash(p)==digest for p,digest in original_hashes.items())
save(OUT/'final_qualification.json',dict(global_capture_passed=False,original_failure_preserved=True,
    contiguous_measurement_window_valid=True,relative_geometry_eligible=True,
    original_inputs_unchanged=True,source_trial=str(RUN/'trial.json'),
    selected_geometry_review=str(RUN/'midpoint_independent_window_v1/midpoint_review.json'),
    motion_window_quality=geometry_result['independent_clock_required_motion_quality'],
    hardware_commands_sent=0,parameter_adoption_allowed=False))
print(json.dumps(dict(global_capture_passed=False,relative_geometry_eligible=True,
    exit_deg=geometry_result['exit_path_direction_change_deg'],Lend=geometry_result['midpoint_Lend_right_forward_mm'],
    residual=geometry_result['midpoint_Lend_residual_mm']),ensure_ascii=False),flush=True)
