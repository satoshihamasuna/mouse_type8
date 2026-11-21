"""保存動画だけを使うターン解析の計測。機器には接続しない。"""
import argparse
import cProfile
import hashlib
import io
import importlib.util
import json
import os
from pathlib import Path
import pstats
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools' / 'analysis'))
sys.path.insert(0, str(ROOT / 'tools'))
import cv2
import numpy as np
import psutil
import pandas as pd
from video_marker_pose import read_video_pose

parser = argparse.ArgumentParser()
parser.add_argument('--trial', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--threads', type=int)
parser.add_argument('--frames', type=int, default=120)
parser.add_argument('--full-window', action='store_true')
parser.add_argument('--no-cprofile', action='store_true')
parser.add_argument('--baseline-source', type=Path)
parser.add_argument('--compare-pose', type=Path)
args = parser.parse_args()
if args.baseline_source:
    spec = importlib.util.spec_from_file_location('turn_pose_baseline', args.baseline_source)
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    read_video_pose = baseline.read_video_pose
args.output.mkdir(parents=True, exist_ok=False)
folder = args.trial.parent
trial = json.loads(args.trial.read_text(encoding='utf-8'))
geometry = json.loads((folder / 'body_geometry_used.json').read_text(encoding='utf-8'))
reference = json.loads((folder / 'marker_reference_used.json').read_text(encoding='utf-8'))
provenance = json.loads((folder / 'full' / 'apparent_floor_marker_pose.provenance.json').read_text(encoding='utf-8'))
start = provenance['video_window'][0]
end = start + args.frames / 240
if args.full_window:
    end = provenance['video_window'][1]
if args.threads is not None:
    cv2.setNumThreads(args.threads)
samples = []
stop = threading.Event()
process = psutil.Process()
psutil.cpu_percent(None)
process.cpu_percent(None)

def sample():
    while not stop.wait(.25):
        memory = psutil.virtual_memory()
        samples.append(dict(cpu_total_percent=psutil.cpu_percent(None),
            cpu_process_percent=process.cpu_percent(None),
            rss_bytes=process.memory_info().rss, available_ram_bytes=memory.available))

sampler = threading.Thread(target=sample, daemon=True)
sampler.start()
profile = cProfile.Profile()
wall_start, cpu_start = time.perf_counter(), time.process_time()
if not args.no_cprofile:
    profile.enable()
pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
    marker_reference=reference, video_window=(start, end),
    marker_detection=geometry.get('marker_detection'), rear_colour=geometry.get('rear_colour', 'yellow'),
    nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
    marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
if not args.no_cprofile:
    profile.disable()
wall, cpu = time.perf_counter() - wall_start, time.process_time() - cpu_start
stop.set()
sampler.join()
if not args.no_cprofile:
    profile.dump_stats(str(args.output / 'profile.pstats'))
pose.to_csv(args.output / 'pose.csv', index=False)
stream = io.StringIO()
if not args.no_cprofile:
    pstats.Stats(profile, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(22)
(args.output / 'profile.txt').write_text(stream.getvalue(), encoding='utf-8')
result = dict(trial=str(args.trial), video=trial['video_file'], window=[start, end],
    frames=len(pose), wall_seconds=wall, process_cpu_seconds=cpu, average_cpu_cores=cpu / wall,
    frames_per_second=len(pose) / wall, opencv_threads=cv2.getNumThreads(),
    logical_cpus=os.cpu_count(), detection_rate=float(pose.marker_detected_raw.mean()),
    cpu_total_percent_mean=float(np.mean([s['cpu_total_percent'] for s in samples])) if samples else None,
    peak_rss_mib=max([s['rss_bytes'] for s in samples], default=0) / 2**20,
    minimum_available_ram_mib=min([s['available_ram_bytes'] for s in samples], default=0) / 2**20,
    source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in
        ['tools/analysis/video_marker_pose.py', 'tools/pixel_camera_reference.py', 'tools/analysis/pixel_camera_accuracy.py']})
result['baseline_source'] = str(args.baseline_source) if args.baseline_source else None
if args.baseline_source:
    result['baseline_source_sha256'] = hashlib.sha256(args.baseline_source.read_bytes()).hexdigest()
result['cprofile_enabled'] = not args.no_cprofile
(args.output / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
if args.compare_pose:
    expected = pd.read_csv(args.compare_pose)
    actual = pd.read_csv(args.output / 'pose.csv')
    pd.testing.assert_frame_equal(expected, actual, check_exact=False, rtol=1e-12, atol=1e-9)
    result['comparison'] = dict(path=str(args.compare_pose), all_columns_match=True,
        rtol=1e-12, atol=1e-9, rows=len(expected), columns=len(expected.columns))
(args.output / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result), flush=True)
print(stream.getvalue())
