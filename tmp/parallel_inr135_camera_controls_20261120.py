"""OFF試写のAF/AE/zoom状態とencoder gapの時刻を保存CSVで比較する。"""
from pathlib import Path
from collections import Counter
import csv
import json
import statistics

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'parallel_analysis/camera_retest_controls_v2'
OUT.mkdir(parents=True, exist_ok=False)
NAMES = [('retest01', '20261120_080731_f7b14807.capture.json'),
         ('retest02_background', '20261120_080913_a49ee8cd.capture.json'),
         ('retest03_focus_unlocked', '20261120_081045_5a9f1954.capture.json')]


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as file:
        return list(csv.DictReader(file))


results = []
for label, filename in NAMES:
    manifest = read(ROOT / 'tools/logs/pixel_camera' / filename)
    metadata_path = Path(manifest['metadata_file'])
    metadata = read(metadata_path)
    sensor = rows(metadata_path.parent / metadata['sensor_csv'])
    encoded = rows(metadata_path.parent / metadata['pts_csv'])
    normal = statistics.median((int(b['sensor_timestamp_ns'])-int(a['sensor_timestamp_ns'])) /
        (int(b['frame_number'])-int(a['frame_number'])) for a,b in zip(sensor,sensor[1:])
        if 0 < int(b['sensor_timestamp_ns'])-int(a['sensor_timestamp_ns']) < 50_000_000)
    origin = int(sensor[0]['sensor_timestamp_ns'])-int(sensor[0]['frame_number'])*normal
    pts = [int(row['presentation_time_ns'])/1e9 for row in encoded]
    gaps = [dict(before_s=a, after_s=b, interval_ms=(b-a)*1000) for a,b in zip(pts,pts[1:]) if b-a>1.5/240]
    fields = ['af_state','ae_state','awb_state','af_mode_result','af_trigger_requested','focus_distance_diopters','zoom_ratio_result','ev_steps_result']
    transitions = {field:[] for field in fields}
    for a,b in zip(sensor,sensor[1:]):
        stamp = (int(b['sensor_timestamp_ns'])-origin)/1e9
        for field in fields:
            if a.get(field) != b.get(field):
                transitions[field].append(dict(relative_s=stamp, frame=int(b['frame_number']), before=a.get(field),after=b.get(field)))
    controls_near_gaps = []
    for gap in gaps:
        before,after=gap['before_s'],gap['after_s']
        nearest = min(sensor, key=lambda row:abs((int(row['sensor_timestamp_ns'])-origin)/1e9-before))
        controls_near_gaps.append(dict(**gap, nearest_frame=int(nearest['frame_number']),
            nearest_sensor={field:nearest.get(field) for field in fields},
            transitions_within_50ms={field:[change for change in changes if before-.05<=change['relative_s']<=after+.05]
                                    for field,changes in transitions.items()}))
    first_gap = gaps[0]['before_s'] if gaps else None
    result=dict(label=label,source_manifest=filename,recording_id=manifest['recording_id'],focus_lock=metadata['focus_lock'],
        state=manifest['state'],decoded_fps=manifest['video_inspection']['fps_from_decoded_pts'],gap_count=len(gaps),
        first_gap_s=first_gap,last_gap_s=gaps[-1]['after_s'] if gaps else None,
        clock_alignment='Approximate Camera2 frame-number origin plus median normal batch cadence; no independent hardware synchronization.',
        normal_frame_period_ms=normal/1e6,
        control_value_counts={field:dict(Counter(row.get(field) for row in sensor)) for field in fields},
        control_transitions=transitions,
        control_transition_counts_after_first_gap={field:sum(change['relative_s']>=first_gap for change in changes) if first_gap is not None else None for field,changes in transitions.items()},
        control_transitions_near_any_gap={field:sum(any(gap['before_s']-.05<=change['relative_s']<=gap['after_s']+.05 for gap in gaps) for change in changes) for field,changes in transitions.items()},
        frame_duration_ms_range=[min(int(row['frame_duration_ns']) for row in sensor)/1e6,max(int(row['frame_duration_ns']) for row in sensor)/1e6],
        exposure_ms_range=[min(int(row['exposure_ns']) for row in sensor)/1e6,max(int(row['exposure_ns']) for row in sensor)/1e6],
        controls_near_gaps=controls_near_gaps)
    results.append(result)
with (OUT/'control_gap_correlation.json').open('x',encoding='utf-8') as file:
    json.dump(dict(results=results,hardware_operations=0,external_repository_modified=False),file,ensure_ascii=False,indent=2)
    file.write('\n')
print(json.dumps([{key:value for key,value in result.items() if key not in ('controls_near_gaps','control_transitions')} for result in results],ensure_ascii=False,indent=2),flush=True)
