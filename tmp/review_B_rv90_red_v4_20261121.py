"""固定済みの淡赤プロファイルで同じ元動画を再解析。元の失敗は保持する。"""
from pathlib import Path
import argparse
from contextlib import redirect_stdout,redirect_stderr
import hashlib
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
VALIDATION=BASE/'rear_red_profile_v4_validation'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',required=True)
args=parser.parse_args()
RUN=(BASE/args.run).resolve();assert RUN.is_relative_to(BASE.resolve())
OUT=RUN/'red_v4_full';assert not OUT.exists()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
trial,settings=read(RUN/'trial.json'),read(RUN/'saved_1ms.settings.json')
assert trial['mouse']==settings['mouse']=='B' and trial['motion']==settings['motion_name']=='r_v90'
assert trial['preset']==settings['preset_speed_mm_s']==1600 and trial['execution_resend_count']==0
validation=read(VALIDATION/'visual_validation.json')
assert validation['approved_for_offline_full_video_validation']
profile=Path(validation['profile']);assert hashlib.sha256(profile.read_bytes()).hexdigest()==validation['profile_sha256']
geometry=read(profile);old_geometry=read(RUN/'body_geometry_used.json')
for key in ('mouse','front_colour','rear_colour','front_height_mm','rear_height_mm','nominal_marker_span_mm','marker_span_limits_mm','reference_id'):
    assert geometry[key]==old_geometry[key],key
assert geometry['marker_detection']['green']==old_geometry['marker_detection']['green']
for key,value in old_geometry['marker_detection']['red'].items():
    if key not in ('hue_ranges','minimum_saturation'):
        assert geometry['marker_detection']['red'][key]==value,key
window=read(RUN/'full_window_choice.json')
start,end=window['video_start_s'],window['video_end_s']
request=dict(source_trial=str(RUN/'trial.json'),geometry_profile=str(profile),geometry_sha256=validation['profile_sha256'],
    original_geometry=str(RUN/'body_geometry_used.json'),original_full_report=str(RUN/'full/report.json'),
    output=str(OUT),video_start_s=start,video_end_s=end,frame_stride=1,
    original_results_preserved=True,physical_quality_guards_unchanged=True,hardware_operations=0)
with (RUN/'red_v4_analysis_request.json').open('x',encoding='utf-8') as f:json.dump(request,f,ensure_ascii=False,indent=2)
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
import cv2
cv2.setNumThreads(4)
from pixel_camera_tuning_report import summarize
import pixel_camera_relative_pose as relative
sys.argv=[str(ROOT/'tools/analysis/pixel_camera_relative_pose.py'),'--trial',str(RUN/'trial.json'),
    '--marker-reference',str(RUN/'marker_reference_used.json'),'--body-geometry',str(profile),
    '--camera-height-mm',str(trial['camera_height_mm']),'--height-source',trial['height_source'],
    '--fixed-camera','--captured-trial','--video-start',str(start),'--video-end',str(end),
    '--frame-stride','1','--output',str(OUT)]
with (RUN/'red_v4_full.stdout.txt').open('x',encoding='utf-8') as output:
    with redirect_stdout(output),redirect_stderr(output):
        code=relative.main()
        if code==0:
            try:summary=summarize(OUT)
            except Exception as error:
                summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
                with (OUT/'summary.json').open('x',encoding='utf-8') as f:json.dump(summary,f,ensure_ascii=False,indent=2)
        else:summary=dict(state='FULL_REVIEW_REQUIRED',returncode=code)
print(json.dumps(dict(full_returncode=code,summary=summary),ensure_ascii=False),flush=True)
if code:raise SystemExit(code)
with (RUN/'red_v4_midpoint.stdout.txt').open('x',encoding='utf-8') as output:
    result=subprocess.run([sys.executable,str(ROOT/'tmp/review_turn1600_B_rv90_midpoint_redv4_20261121.py'),args.run],
        stdout=output,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONUTF8='1'),creationflags=subprocess.CREATE_NO_WINDOW)
if result.returncode:raise SystemExit(result.returncode)
midpoint=read(RUN/'red_v4_midpoint_clock_v1/midpoint_review.json')
print(json.dumps({key:midpoint[key] for key in ('independent_optical_geometry_eligible',
    'exit_path_direction_change_deg','midpoint_Lend_residual_mm','length_only_candidate_at_current_angle')},ensure_ascii=False),flush=True)
