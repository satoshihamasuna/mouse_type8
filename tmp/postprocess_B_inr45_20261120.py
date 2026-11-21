"""B1600 inR45の取得済み一試行を全フレーム・1msログ・入口時計で評価する。機器操作なし。"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('run_relative_to_base')
args=parser.parse_args()
RUN=(BASE/args.run_relative_to_base).resolve()
assert RUN.is_relative_to(BASE)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial=read(RUN/'trial.json')
assert trial['mouse']=='B' and trial['motion']=='in_r45' and trial['preset']==1600
assert trial['state']=='CAPTURED_REVIEW_REQUIRED' and trial['execution_resend_count']==0
assert trial['saved_log']['status']=='saved' and trial['saved_log']['drop_count']==0
assert not (RUN/'full').exists() and not (RUN/'tracking_1ms_review_v3').exists()
env=dict(os.environ,PYTHONUTF8='1',OPENCV_FOR_THREADS_NUM='4')
flags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0
start=max(0.,trial['video_motion_start_estimate_s']-.35)
end=trial['recording_stop']['confirmation_camera_seconds']+.65
assert end>start+1
with (RUN/'full_window_request.json').open('x',encoding='utf-8') as f:
    json.dump(dict(video_start=start,video_end=end,frame_stride=1,
        scope='動作前静止、動作全区間、後半静止を全フレーム評価。後続のファイル確定待ちを除外。'),f,ensure_ascii=False,indent=2)
with (RUN/'tracking.stdout.txt').open('x',encoding='utf-8') as tracklog:
    tracking=subprocess.Popen([sys.executable,str(ROOT/'tmp/review_B_inr45_tracking_20261120.py'),args.run_relative_to_base],
        stdout=tracklog,stderr=subprocess.STDOUT,env=env,creationflags=flags)
    with (RUN/'full.stdout.txt').open('x',encoding='utf-8') as fulllog:
        full=subprocess.run([sys.executable,str(ROOT/'tools/analysis/pixel_camera_tuning_report.py'),
            '--trial',str(RUN/'trial.json'),'--stride','1','--video-start',str(start),'--video-end',str(end)],
            stdout=fulllog,stderr=subprocess.STDOUT,env=env,creationflags=flags)
    track_code=tracking.wait()
assert track_code==0
assert (RUN/'full/report.json').is_file(), '全フレーム解析のreport.jsonがありません'
with (RUN/'midpoint.stdout.txt').open('x',encoding='utf-8') as log:
    midpoint=subprocess.run([sys.executable,str(ROOT/'tmp/review_B_inr45_midpoint_20261120.py'),args.run_relative_to_base],
        stdout=log,stderr=subprocess.STDOUT,env=env,creationflags=flags)
assert midpoint.returncode==0
geometry=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
summary=read(RUN/'full/summary.json')
result=dict(state='FULL_EVALUATED_REVIEW_REQUIRED',full_returncode=full.returncode,
    tracking_returncode=track_code,midpoint_returncode=midpoint.returncode,
    independent_optical_geometry_eligible=geometry['independent_optical_geometry_eligible'],
    auto_stop_confirmed=summary['auto_stop_confirmed'],
    selected_geometry_review=str(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),
    quick_skipped='全フレームを評価済み。速報は重複実行しない。',archive_state='PENDING',
    parameter_adoption_allowed=False,hardware_commands_sent=0)
with (RUN/'postprocess_status.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
pending=read(RUN/'deferred_analysis.json');pending.update(result)
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
