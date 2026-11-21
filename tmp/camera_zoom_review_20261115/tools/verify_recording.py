"""Verify a Phase 1 export using SHA-256, raw sensor CSV and independently decoded frames."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess


def timing(timestamps: list[float]) -> dict:
    deltas = [b - a for a, b in zip(timestamps, timestamps[1:])]
    span = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
    return {
        "count": len(timestamps),
        "span_seconds": span,
        "fps_from_timestamps": (len(timestamps) - 1) / span if span > 0 else None,
        "median_interval_ms": statistics.median(deltas) * 1000 if deltas else None,
        "gaps_over_1_5_frames": sum(delta > 1.5 / 240 for delta in deltas),
        "non_increasing_timestamps": sum(delta <= 0 for delta in deltas),
    }


def sibling(metadata_path: Path, name: str) -> Path:
    # Android emits flat artifact names. Reject a metadata reference to another directory.
    if not isinstance(name, str) or not name or name in (".", "..") or "/" in name or "\\" in name or ":" in name:
        raise ValueError(f"Invalid artifact name: {name!r}")
    return metadata_path.parent / name


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def sensor_sampling(rows: list[dict], burst_size: int) -> dict:
    if len(rows) < 2 or any(not row.get("frame_number") or not row.get("sensor_timestamp_ns") for row in rows):
        return {"mode": "UNAVAILABLE", "frame_number_step": None, "fps_from_frame_number_span_estimate": None}
    numbers = [int(row["frame_number"]) for row in rows]
    timestamps = [int(row["sensor_timestamp_ns"]) for row in rows]
    steps = [b - a for a, b in zip(numbers, numbers[1:])]
    step = steps[0] if steps[0] > 0 and len(set(steps)) == 1 else None
    span = timestamps[-1] - timestamps[0]
    mode = ("EVERY_FRAME" if step == 1 else
            "ONE_CALLBACK_PER_BATCH" if burst_size > 1 and step == burst_size else "IRREGULAR")
    return {
        "mode": mode, "frame_number_step": step,
        "fps_from_frame_number_span_estimate": (numbers[-1] - numbers[0]) * 1e9 / span if span > 0 else None,
        "estimated_covered_frames": numbers[-1] - numbers[0] + step if step is not None else None,
        "individual_sensor_frames_verified": False,
    }


def verify(metadata_path: Path, ffprobe: str = "ffprobe", ffmpeg: str | None = None) -> dict:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    video = sibling(metadata_path, metadata["video_file"])
    errors = []
    warnings = []
    if metadata.get("state") != "FINALIZED":
        errors.append("ANDROID_RECORDING_NOT_FINALIZED")
    size_matches = video.stat().st_size == metadata.get("video_size_bytes")
    hash_matches = sha256(video) == metadata.get("sha256")
    if not size_matches or not hash_matches:
        errors.append("SIZE_OR_SHA256_MISMATCH")
    # show_frames decodes the stream; frame count is not taken from the MP4's advertised FPS.
    command = [ffprobe, "-v", "error", "-select_streams", "v:0",
               "-show_frames", "-show_streams", "-show_entries",
               "stream=codec_name,width,height,avg_frame_rate,r_frame_rate,duration:frame=best_effort_timestamp_time",
               "-of", "json", str(video)]
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", timeout=180, check=True)
    decoded = json.loads(result.stdout)
    if result.stderr.strip():
        errors.append("DECODER_REPORTED_ERRORS")
    streams = decoded.get("streams", [])
    if not streams:
        raise ValueError("No video stream found")
    stream = streams[0]
    if stream.get("width") != 1280 or stream.get("height") != 720 or stream.get("codec_name") != "h264":
        errors.append("UNEXPECTED_CODEC_OR_RESOLUTION")
    frames = decoded.get("frames", [])
    pts = [float(f["best_effort_timestamp_time"]) for f in frames if "best_effort_timestamp_time" in f]
    if len(pts) != len(frames) or not all(math.isfinite(value) for value in pts):
        raise ValueError("Some decoded frames do not have finite timestamps")
    encoded = timing(pts)
    expected_count = metadata.get("encoded_timing", {}).get("count")
    if len(frames) != expected_count:
        errors.append("DECODED_FRAME_COUNT_MISMATCH")
    sensor_file = sibling(metadata_path, metadata["sensor_csv"])
    with sensor_file.open(encoding="utf-8", newline="") as source:
        rows = list(csv.DictReader(source))
    sensor = timing([int(row["sensor_timestamp_ns"]) / 1e9 for row in rows if row["sensor_timestamp_ns"]])
    if sensor["count"] != len(rows):
        errors.append("MISSING_SENSOR_TIMESTAMPS")
    sampling = sensor_sampling(rows, metadata.get("high_speed_burst_size", 1))
    batched = sampling["mode"] == "ONE_CALLBACK_PER_BATCH"
    if batched:
        warnings.append("INCONCLUSIVE_BATCHED_SENSOR_METADATA")
    elif sampling["mode"] == "IRREGULAR":
        warnings.append("INCONCLUSIVE_METADATA_SAMPLING")
    decoded_rate = encoded["fps_from_timestamps"]
    decoded_timing_consistent = (
        decoded_rate is not None and abs(decoded_rate - 240) <= 4.8
        and not encoded["gaps_over_1_5_frames"] and not encoded["non_increasing_timestamps"]
    )
    if not decoded_timing_consistent:
        errors.append("DECODED_RATE_OR_TIMESTAMP_GAPS")
    if sampling["mode"] != "IRREGULAR":
        sensor_rate = sampling["fps_from_frame_number_span_estimate"] if batched else sensor["fps_from_timestamps"]
        if sensor_rate is None or abs(sensor_rate - 240) > 4.8:
            errors.append("SENSOR_RATE_NOT_NEAR_240")
        if sensor["non_increasing_timestamps"] or (not batched and sensor["gaps_over_1_5_frames"]):
            errors.append("SENSOR_TIMESTAMP_GAPS")
        expected_sensor_frames = sampling["estimated_covered_frames"] if batched else sensor["count"]
        tolerance = sampling["frame_number_step"] if batched else 3
        if abs(expected_sensor_frames - encoded["count"]) > tolerance:
            errors.append("SENSOR_DECODED_COUNT_DIFFER")
    # Raw callback rate is not capture FPS when one callback represents an entire batch.
    sensor["interpretation"] = "callback_cadence_not_capture_fps"
    if sampling["mode"] != "EVERY_FRAME":
        sensor.pop("gaps_over_1_5_frames", None)
    if sensor["span_seconds"] <= 0 or not 0.95 <= encoded["span_seconds"] / sensor["span_seconds"] <= 1.05:
        errors.append("SENSOR_DECODED_DURATION_DIFFER")
    wall = metadata.get("control_elapsed_seconds", 0)
    if wall < 2:
        errors.append("INCONCLUSIVE_SHORT_RECORDING")
    elif any(abs(item["span_seconds"] - wall) > max(0.5, wall * 0.1) for item in (sensor, encoded)):
        errors.append("WALL_CLOCK_DURATION_MISMATCH")
    if metadata.get("capture_failures", 0) or metadata.get("buffer_losses", 0):
        errors.append("ANDROID_CAPTURE_OR_BUFFER_LOSS")
    duplicate_info = None
    if ffmpeg:
        # Optional exact decoded-pixel comparison. A static scene can have identical frames.
        output = subprocess.run([ffmpeg, "-v", "error", "-i", str(video), "-map", "0:v:0",
                                 "-an", "-f", "framemd5", "-"], capture_output=True, text=True,
                                encoding="utf-8", timeout=180, check=True)
        hashes = [line.rsplit(",", 1)[-1].strip() for line in output.stdout.splitlines()
                  if line.strip() and not line.startswith("#")]
        duplicate_info = {
            "decoded_frames": len(hashes),
            "consecutive_identical_pixel_frames": sum(a == b for a, b in zip(hashes, hashes[1:])),
            "note": "A static scene can repeat. Different pixels alone do not prove physical 240 fps.",
        }
        if output.stderr.strip():
            errors.append("DUPLICATE_CHECK_DECODER_ERRORS")
        if len(hashes) != len(frames):
            errors.append("DUPLICATE_CHECK_FRAME_COUNT_MISMATCH")
    return {
        "file": str(video), "size_matches": size_matches, "sha256_matches": hash_matches,
        "container_declarations_only": stream, "sensor": sensor, "decoded": encoded,
        "wall_elapsed_seconds": wall, "duplicate_check": duplicate_info,
        "sensor_sampling": sampling, "decoded_timing_consistent": decoded_timing_consistent,
        "timing_checks_passed": not errors and not warnings, "errors": errors, "warnings": warnings,
        "physical_240fps_verified": False,
        "required_next_check": "Inspect a moving target with an independent known time reference. Camera2 high-speed timestamps may be interpolated in batches.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metadata", type=Path, help="Recording JSON; not capabilities.json")
    parser.add_argument("--ffprobe", default="ffprobe")
    parser.add_argument("--ffmpeg", help="Optional ffmpeg executable for exact duplicate-pixel checks")
    args = parser.parse_args()
    try:
        report = verify(args.metadata, args.ffprobe, args.ffmpeg)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        parser.exit(2, f"Verification could not finish: {error}\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["timing_checks_passed"] else 1)


if __name__ == "__main__":
    main()
