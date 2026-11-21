"""今回B1600右135度出口の保存済み動画だけを全フレーム解析する。機器操作なし。"""
from pathlib import Path
import argparse
from contextlib import redirect_stdout, redirect_stderr
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',required=True,help='BASEからの試行相対パス')
args=parser.parse_args()
run=(BASE/args.run).resolve()
assert run.is_relative_to(BASE.resolve())
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
trial,settings=read(run/'trial.json'),read(run/'saved_1ms.settings.json')
assert trial['mouse']==settings['mouse']=='B' and trial['motion']==settings['motion_name']=='out_r135'
assert trial['preset']==settings['preset_speed_mm_s']==1600
assert trial['camera_state']=='PASSED' and trial['state']=='CAPTURED_REVIEW_REQUIRED'
assert trial['execution_resend_count']==0 and trial['turn_count']==1
assert not (run/'full').exists()
from pixel_camera_tuning_report import analysis_video_window,summarize
import pixel_camera_relative_pose as relative
import cv2
cv2.setNumThreads(4)
default_start,end=analysis_video_window(trial)
default_end=end
stop=trial.get('recording_stop',{})
if stop.get('request_stop') is True and stop.get('decision_camera_seconds') is not None:
    end=min(end,float(stop['decision_camera_seconds'])+.2)
elif stop.get('request_stop') is False:
    # 上限まで続いた動画の末尾を避ける。実動全区間と前後静止窓は
    # 後段で実測時刻・全フレーム観測率・静止ばらつきから検証する。
    end=min(end,trial['video_motion_start_estimate_s']+4.5)
# 同じB1600/吸引650の基準動画ではRUN_START推定から実動開始まで約1.4秒。
# 余白を残して解析開始を+0.6秒に置き、全走行・前後の観測条件は解析結果で検査する。
assert settings['suction']==dict(enabled=True,duty=650)
start=max(default_start,trial['video_motion_start_estimate_s']+.6)
with (run/'full_window_choice.json').open('x',encoding='utf-8') as file:
    json.dump(dict(video_start_s=start,video_end_s=end,original_default_start_s=default_start,original_default_end_s=default_end,
        frame_stride=1,cv2_threads=4,raw_video_modified=False,
        basis='同条件の初回実動遅延を参考に吸引待ちの前半を除外。必要走行区間の包含・観測95%・各フレーム実ArUco登録と前後静止観測の条件は維持。',
        baseline_review=str(ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/baseline002_review.json')),file,ensure_ascii=False,indent=2)
sys.argv=[str(ROOT/'tools/analysis/pixel_camera_relative_pose.py'),'--trial',str(run/'trial.json'),
    '--marker-reference',str(run/'marker_reference_used.json'),'--body-geometry',str(run/'body_geometry_used.json'),
    '--camera-height-mm',str(trial['camera_height_mm']),'--height-source',trial['height_source'],
    '--fixed-camera','--captured-trial','--video-start',str(start),'--video-end',str(end),
    '--frame-stride','1','--output',str(run/'full')]
with (run/'full.stdout.txt').open('x',encoding='utf-8') as output:
    with redirect_stdout(output),redirect_stderr(output):
        code=relative.main()
        if code==0:
            try:
                summary=summarize(run/'full')
            except Exception as error:
                summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
                with (run/'full/summary.json').open('x',encoding='utf-8') as file:
                    json.dump(summary,file,ensure_ascii=False,indent=2)
        else: summary=dict(state='FULL_REVIEW_REQUIRED',returncode=code)
print(json.dumps(dict(full_returncode=code,summary=summary),ensure_ascii=False),flush=True)
if code:raise SystemExit(code)
with (run/'midpoint.stdout.txt').open('x',encoding='utf-8') as output:
    result=subprocess.run([sys.executable,str(ROOT/'tmp/review_turn1600_B_outr135_midpoint_20261121.py'),args.run],
        stdout=output,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONUTF8='1'),creationflags=subprocess.CREATE_NO_WINDOW)
if result.returncode:raise SystemExit(result.returncode)
midpoint=read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
print(json.dumps({key:midpoint[key] for key in ('independent_optical_geometry_eligible',
    'exit_path_direction_change_deg','midpoint_Lend_right_forward_mm','length_only_candidate_at_current_angle')},ensure_ascii=False),flush=True)
