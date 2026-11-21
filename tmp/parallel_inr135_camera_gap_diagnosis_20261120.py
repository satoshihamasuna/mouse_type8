"""保存済みCamera2/encoder時刻だけを比較する。ADB・撮影・外部repo編集なし。"""
from pathlib import Path
from collections import Counter
import csv
import json
import statistics

ROOT = Path(__file__).resolve().parents[1]
REMOTE = Path('D:/TYPE8_Camera/recordings/remote')
OUT = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/parallel_analysis/camera_gap_diagnosis_v2'
OUT.mkdir(parents=True, exist_ok=True)
assert not any(OUT.iterdir()), '既存出力を上書きしない'
names = [('last_known_good', '20261120_031447_240fps_cb61eab9'),
         ('static001', '20261120_075637_240fps_88c556d9'),
         ('static002', '20261120_080238_240fps_16485682'),
         ('static003', '20261120_080413_240fps_d99d23e6')]


def csvrows(path):
    with path.open(encoding='utf-8-sig', newline='') as file:
        return list(csv.DictReader(file))


results = []
for label, name in names:
    metadata = json.loads((REMOTE / (name+'.json')).read_text(encoding='utf-8-sig'))
    sensor = csvrows(REMOTE / (name+'.sensor.csv'))
    points = csvrows(REMOTE / (name+'.pts.csv'))
    # Camera2 SENSOR_TIMESTAMP と MediaCodec sourcePTSは時計原点が異なる。
    # 先頭frame番号と通常バッチ周期から概略対応だけを作り、独立同期とは扱わない。
    periods = [(int(b['sensor_timestamp_ns'])-int(a['sensor_timestamp_ns'])) /
        (int(b['frame_number'])-int(a['frame_number']))
        for a,b in zip(sensor,sensor[1:]) if 0 < int(b['sensor_timestamp_ns'])-int(a['sensor_timestamp_ns']) < 50_000_000]
    normal_period = statistics.median(periods)
    origin = int(sensor[0]['sensor_timestamp_ns'])-int(sensor[0]['frame_number'])*normal_period
    pts = [int(row['presentation_time_ns'])/1e9 for row in points]
    gaps = [dict(before_s=a, after_s=b, interval_ms=(b-a)*1000)
            for a,b in zip(pts,pts[1:]) if b-a > 1.5/240]
    intervals = []
    for a,b in zip(sensor,sensor[1:]):
        before = (int(a['sensor_timestamp_ns'])-origin)/1e9
        after = (int(b['sensor_timestamp_ns'])-origin)/1e9
        if after-before > 1.5*8/240:
            intervals.append(dict(before_frame=int(a['frame_number']), after_frame=int(b['frame_number']),
                frame_step=int(b['frame_number'])-int(a['frame_number']), before_s=before, after_s=after,
                interval_ms=(after-before)*1000, frame_duration_ms=int(b['frame_duration_ns'])/1e6,
                exposure_ms=int(b['exposure_ns'])/1e6, af_state=int(b['af_state']),
                af_trigger_requested=int(b['af_trigger_requested'])))
    trigger = metadata['focus_lock']['trigger_frame']
    trigger_row = next((row for row in sensor if int(row['frame_number']) == trigger), None)
    trigger_s = (int(trigger_row['sensor_timestamp_ns'])-origin)/1e9 if trigger_row else None
    result = dict(label=label, recording_id=name, encoder=metadata['encoder'],bitrate=metadata['bitrate'],
        sensor_frame_step_counts=dict(Counter(int(b['frame_number'])-int(a['frame_number']) for a,b in zip(sensor,sensor[1:]))),
        clock_alignment='Approximate: initial Camera2 frame index and normal batch cadence only; sensor and codec raw origins differ.',
        sensor_long_intervals=intervals, encoded_gaps=gaps,
        sensor_fps_estimate=metadata['sensor_sampling']['fps_from_frame_number_span_estimate'],
        encoded_fps=metadata['encoded_timing']['fps_from_timestamps'],
        focus_trigger_frame=trigger, focus_trigger_relative_s=trigger_s,
        nearest_encoded_gap_to_focus_trigger_ms=min(abs(gap['before_s']-trigger_s)*1000 for gap in gaps) if gaps and trigger_s is not None else None,
        frame_duration_ms_range=[min(int(row['frame_duration_ns']) for row in sensor)/1e6,max(int(row['frame_duration_ns']) for row in sensor)/1e6],
        exposure_ms_range=[min(int(row['exposure_ns']) for row in sensor)/1e6,max(int(row['exposure_ns']) for row in sensor)/1e6],
        iso_range=[min(int(row['iso']) for row in sensor),max(int(row['iso']) for row in sensor)],
        camera_capture_failures=metadata['capture_failures'],camera_buffer_losses=metadata['buffer_losses'],
        source_sensor_csv=str(REMOTE/(name+'.sensor.csv')),source_pts_csv=str(REMOTE/(name+'.pts.csv')),
        sensor_and_encoder_gaps_are_time_correlated=all(any(interval['before_s']<=gap['before_s']+.010 and interval['after_s']>=gap['after_s']-.010 for interval in intervals) for gap in gaps))
    results.append(result)
with (OUT/'timing_comparison.json').open('x',encoding='utf-8') as file:
    json.dump(dict(results=results,hardware_operations=0,external_repository_modified=False,
        supersedes='camera_gap_diagnosis_v1: sensor versus codec raw timestamp origins differ; reject its raw-origin subtraction.'),file,ensure_ascii=False,indent=2)
    file.write('\n')
print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)
