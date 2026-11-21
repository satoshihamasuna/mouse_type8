"""録画が上限まで続いた取得済み反復の後半静止窓だけを別版で評価。機器操作なし。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source_path=ROOT/'tmp/review_turn1600_B_inr135_midpoint_20261120.py'
source=source_path.read_text(encoding='utf-8-sig')
needle="steady_window=[trial['recording_stop']['confirmation_camera_seconds']+.10,trial['recording_stop']['confirmation_camera_seconds']+.54]"
assert source.count(needle)==1
source=source.replace(needle,"""confirmation=trial['recording_stop'].get('confirmation_camera_seconds')
if confirmation is not None:
    steady_window=[confirmation+.10,confirmation+.54]
else:
    assert trial['recording_stop']['request_stop'] is False
    assert trial['log_stop_evidence']['confirmed'] is True
    # 保存済みの連続CSVと操作者のOFF確認を保持し、録画末尾を別の静止根拠とする。
    steady_window=[provenance['video_window'][1]-.65,provenance['video_window'][1]-.15]""")
source=source.replace("OUT=RUN/'midpoint_raw_wheel_clock_v1'","OUT=RUN/'midpoint_raw_wheel_clock_fallback_v2'")
source=source.replace("late_stationary=dict(window_pts_s=steady_window", "recording_stop_fallback=trial['recording_stop'],late_stationary=dict(window_pts_s=steady_window")
compile(source,'<fallback stop review>','exec')
target=ROOT/'tmp/review_B_inr135_stop_fallback_20261120.py'
assert not target.exists();target.write_text(source,encoding='utf-8',newline='\n')
print('Prepared separate fallback-window review; original source/quality retained')
