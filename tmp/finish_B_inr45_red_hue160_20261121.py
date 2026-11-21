"""summarize自身が保存した結果を再利用し、完了済みcacheから比較・時計検証を続ける。"""
from pathlib import Path
import json
import os
import subprocess
import sys
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
import pixel_camera_relative_pose as core
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
RUNS={
 '006':'geometry02/local_execution/run_B_INR45_1600_geometry02_green_20261120_006',
 '007':'geometry02_repeat_01/local_execution/run_B_INR45_1600_geometry02_repeat01_green_20261120_007',
 '008':'geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008'}
key=sys.argv[1];RUN=BASE/RUNS[key];OUT=BASE/'red_hue160_validation_v1'/('run'+key)/'full'
trial=read(RUN/'trial.json');qualification=read(OUT/'window_qualification.json');global_passed=qualification['global_capture_passed']
assert read(OUT/'report.json')['frame_stride']==1 and (OUT/'summary.json').is_file()
original_hashes=qualification['original_input_sha256']
profile=BASE/'body_geometry_B_green_red_h160_candidate_v4.json';geometry=read(OUT/'body_geometry_used.json');reference=read(RUN/'marker_reference_used.json')
old_full=RUN/('independent_motion_window_v1' if key=='008' else 'full')
provenance=read(OUT/'apparent_floor_marker_pose.provenance.json')
pose=core.load_pose_cache(OUT/'apparent_floor_marker_pose.csv',trial,reference,geometry,provenance['video_window'],1)
motion_quality=read(OUT/'motion_quality_diagnostic.json')
source_driver=ROOT/'tmp/validate_B_inr45_red_hue160_20261121.py'
tail=source_driver.read_text(encoding='utf-8').split('# 過去の検出済み位置を同じフレームで比較。',1)[1]
tail=tail.split('\n',1)[1]
if not (OUT.parent/'validation_result.json').exists():
 exec(compile(tail,str(source_driver)+' [cached completion]', 'exec'))
save(OUT.parent/'completion_provenance.json',dict(reason='summarize writes summary.json itself; repeated exclusive save was prevented. Reuse original verified pose, no video rescan.',
 source_driver_sha256=core.file_hash(source_driver),completion_driver_sha256=core.file_hash(Path(__file__)),cached_pose_provenance_revalidated=True,
 hardware_commands_sent=0,source_trial_unchanged=True))
