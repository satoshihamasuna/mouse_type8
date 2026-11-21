"""失敗判定を保持し、保存動画のPTS間隔を独立に再確認する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import av

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008'
OUT=RUN/'pts_gap_review_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial=read(RUN/'trial.json');capture=read(Path(trial['capture_json']))
video=Path(trial['video_file']);digest=hashlib.sha256(video.read_bytes()).hexdigest()
assert digest==capture['sha256']==capture['video_inspection']['sha256']
assert video.stat().st_size==capture['size_bytes']
pts=[];gaps=[];corrupt=missing=wrong_size=non_increasing=0
with av.open(str(video)) as container:
    stream=container.streams.video[0];stream.codec_context.thread_count=4
    for index,frame in enumerate(container.decode(stream)):
        corrupt+=bool(frame.is_corrupt)
        wrong_size+=(frame.width,frame.height)!=(1280,720)
        if frame.pts is None:
            missing+=1;continue
        stamp=float(frame.pts*frame.time_base)
        if pts:
            interval=stamp-pts[-1]
            non_increasing+=interval<=0
            if interval>1.5/240:
                gaps.append(dict(before_frame=index-1,after_frame=index,before_pts_s=pts[-1],after_pts_s=stamp,interval_ms=interval*1000))
        pts.append(stamp)
assert len(pts)==capture['video_inspection']['decoded_frames']
result=dict(source_trial=str(RUN/'trial.json'),source_capture=trial['capture_json'],video=str(video),
    original_trial_state=trial['state'],original_capture_state=capture['state'],
    original_inspection_errors=capture['video_inspection']['errors'],original_failure_preserved=True,
    decoded_frames=len(pts),first_pts_s=pts[0],last_pts_s=pts[-1],gaps=gaps,
    corrupt_frames=corrupt,missing_pts=missing,non_increasing_pts=non_increasing,wrong_size_frames=wrong_size,
    video_sha256=digest,size_bytes=video.stat().st_size,hash_and_size_rechecked=True,
    sampling_interval_nominal_ms=1000/240,global_recording_passed=False,
    motion_window_requires_separate_validation=True,hardware_commands_sent=0)
with (OUT/'inspection.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
with (OUT/'decoded_pts.json').open('x',encoding='utf-8') as f:json.dump(pts,f)
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
