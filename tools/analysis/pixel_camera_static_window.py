"""静止5秒動画の3.3..4.95秒だけの復号時刻を別判定する。元manifestは変更しない。"""
from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path


STATIC_WINDOW_S = (3.3, 4.95)
MAX_INTERVAL_S = 1.5 / 240
FRAME_SIZE = (1280, 720)


def file_sha256(path):
    digest = sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def decode_video_frames(path):
    """全フレームを復号する。窓外の破損・欠損・逆行も検出対象に残す。"""
    import av

    with av.open(str(path)) as container:
        for frame in container.decode(video=0):
            yield dict(pts_s=None if frame.pts is None else float(frame.pts * frame.time_base),
                       corrupt=bool(frame.is_corrupt), width=frame.width, height=frame.height)


def assess_decoded_pts(frames):
    """静止専用の固定窓を、前後の境界を跨ぐ間隔も含めて検査する。"""
    frames = list(frames)
    failures = []
    times = [frame['pts_s'] for frame in frames]
    missing = sum(value is None or not math.isfinite(value) for value in times)
    corrupt = sum(bool(frame['corrupt']) for frame in frames)
    wrong_size = sum((frame['width'], frame['height']) != FRAME_SIZE for frame in frames)
    if missing:
        failures.append('missing_or_nonfinite_pts')
    if corrupt:
        failures.append('corrupt_frames')
    if wrong_size:
        failures.append('wrong_size_frames')
    gaps, relevant_intervals, non_increasing = [], [], 0
    start, end = STATIC_WINDOW_S
    if not missing:
        for index, (before, after) in enumerate(zip(times, times[1:])):
            interval = after-before
            if interval <= 0:
                non_increasing += 1
            intersects = before <= end and after >= start
            if intersects:
                relevant_intervals.append(interval)
            if interval > MAX_INTERVAL_S + 1e-12:
                gaps.append(dict(before_frame=index, after_frame=index+1, before_s=before,
                                 after_s=after, interval_ms=interval*1000,
                                 intersects_static_window=intersects))
        if non_increasing:
            failures.append('non_increasing_pts')
        covered = bool(times and times[0] <= start and times[-1] >= end)
        if not covered:
            failures.append('static_window_not_fully_covered')
        if any(gap['intersects_static_window'] for gap in gaps):
            failures.append('pts_gap_inside_or_crossing_static_window')
    else:
        covered = False
    selected = [value for value in times if value is not None and math.isfinite(value) and start <= value <= end]
    if len(selected) < 2:
        failures.append('insufficient_static_window_frames')
    return dict(passed=not failures, failure_reasons=failures, window_s=list(STATIC_WINDOW_S),
                max_allowed_interval_ms=MAX_INTERVAL_S*1000, decoded_frames=len(frames),
                static_window_frames=len(selected), full_window_covered=covered,
                first_pts_s=times[0] if times and not missing else None,
                last_pts_s=times[-1] if times and not missing else None,
                static_window_first_pts_s=selected[0] if selected else None,
                static_window_last_pts_s=selected[-1] if selected else None,
                max_interval_touching_static_window_ms=max(relevant_intervals)*1000 if relevant_intervals else None,
                gaps=gaps, missing_pts=missing, corrupt_frames=corrupt,
                wrong_size_frames=wrong_size, non_increasing_pts=non_increasing)


def inspect_static_window(manifest_path):
    """限定的な静止PTS判定。位置/向きV2および走行動画の判定を置き換えない。"""
    manifest_path = Path(manifest_path)
    original = manifest_path.read_bytes()
    manifest = json.loads(original.decode('utf-8-sig'))
    inspection = manifest.get('video_inspection', {})
    failures = []
    result = dict(schema_version=1, purpose='STATIC_SETTLED_WINDOW_TIMING_ONLY',
                  source_manifest=str(manifest_path.resolve()), source_manifest_state=manifest.get('state'),
                  source_manifest_sha256=sha256(original).hexdigest(), window_s=list(STATIC_WINDOW_S),
                  passed=False, failure_reasons=failures, physical_240fps_verified=False,
                  pose_acceptance_replaced=False, motion_acceptance_replaced=False,
                  original_manifest_reclassified=False)
    if (manifest.get('requested_seconds') != 5 or not str(manifest.get('label', '')).endswith('_static')
            or manifest.get('type8_log_csv') is not None or manifest.get('capture_profile') != 'tuning'):
        failures.append('not_a_five_second_tuning_static_capture')
    errors = inspection.get('errors')
    result['whole_capture_errors_retained'] = errors
    if manifest.get('error') or manifest.get('errors'):
        failures.append('whole_manifest_error_not_allowed')
    if not isinstance(errors, list) or any(error != 'DECODE_OR_PTS_ERROR' for error in errors):
        failures.append('whole_capture_error_not_allowed')
    if manifest.get('state') not in ('PASSED', 'FAILED'):
        failures.append('capture_state_not_final')
    if manifest.get('state') == 'FAILED' and errors != ['DECODE_OR_PTS_ERROR']:
        failures.append('failed_capture_is_not_only_pts_error')
    for name in ('corrupt_frames', 'missing_pts', 'non_increasing_pts', 'wrong_size_frames'):
        if type(inspection.get(name)) is not int or inspection[name] != 0:
            failures.append('whole_capture_' + name)
    expected = manifest.get('sha256')
    if (inspection.get('sha256_rechecked') is not True or not isinstance(expected, str)
            or len(expected) != 64 or expected != inspection.get('sha256')):
        failures.append('transfer_sha_not_verified')
    video_value = manifest.get('video_file')
    if not isinstance(video_value, str) or not video_value:
        failures.append('video_path_missing')
    if failures:
        return result
    video = Path(video_value)
    result['video_file'] = str(video)
    if str(inspection.get('file_path')) != video_value:
        failures.append('inspection_video_path_mismatch')
        return result
    try:
        actual_sha = file_sha256(video)
        result['video_sha256'] = actual_sha
        if actual_sha != expected:
            failures.append('video_sha_mismatch')
        if video.stat().st_size != manifest.get('size_bytes') or video.stat().st_size != inspection.get('size_bytes'):
            failures.append('video_size_mismatch')
        if not failures:
            timing = assess_decoded_pts(decode_video_frames(video))
            result['decoded_timing'] = timing
            failures.extend(timing['failure_reasons'])
            if timing['decoded_frames'] != inspection.get('decoded_frames'):
                failures.append('decoded_frame_count_mismatch')
        if file_sha256(video) != actual_sha:
            failures.append('video_changed_during_inspection')
    except Exception as error:
        failures.append('video_read_or_decode_error')
        result['error'] = f'{type(error).__name__}: {error}'
    if manifest_path.read_bytes() != original:
        failures.append('source_manifest_changed_during_inspection')
    result['passed'] = not failures
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = inspect_static_window(args.manifest)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(passed=result['passed'], failure_reasons=result['failure_reasons'],
                         report=str(args.output)), ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
