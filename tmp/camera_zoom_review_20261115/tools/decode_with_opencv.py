"""Supplementary decoded-frame count and transfer checks when ffprobe is unavailable.

This does not establish per-frame sensor timing. OpenCV position/FPS properties
are not used to certify real-time 240 fps.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from verify_recording import sha256, sibling


def inspect(metadata_path: Path) -> dict:
    import cv2
    import numpy as np

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    video = sibling(metadata_path, metadata["video_file"])
    capture = cv2.VideoCapture(str(video), cv2.CAP_FFMPEG)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open video: {video}")
    count = 0
    identical = 0
    previous = None
    dimensions = set()
    try:
        # Disable automatic rotation: the recorded stream is landscape 1280x720.
        capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
        backend = capture.getBackendName()
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            dimensions.add((frame.shape[1], frame.shape[0]))
            if previous is not None and np.array_equal(previous, frame):
                identical += 1
            previous = frame
            count += 1
    finally:
        capture.release()
    expected = metadata.get("encoded_timing", {}).get("count")
    result = {
        "file": str(video), "decoder_backend": backend, "opencv_version": cv2.__version__,
        "size_matches": video.stat().st_size == metadata.get("video_size_bytes"),
        "sha256_matches": sha256(video) == metadata.get("sha256"),
        "decoded_frame_count": count, "expected_encoded_frame_count": expected,
        "decoded_frame_count_matches": count == expected,
        "decoded_dimensions": sorted(dimensions),
        "consecutive_identical_decoded_frames": identical,
        "per_frame_timing_independently_verified": False,
        "physical_240fps_verified": False,
        "note": "Successful decoding and nonidentical pixels alone do not prove physical capture rate. ffprobe timestamps and a timed moving target remain required.",
    }
    result["integrity_and_decode_checks_passed"] = (
        result["size_matches"] and result["sha256_matches"] and count == expected and count > 0
        and dimensions == {(1280, 720)} and metadata.get("state") == "FINALIZED"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metadata", type=Path)
    args = parser.parse_args()
    result = inspect(args.metadata)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["integrity_and_decode_checks_passed"] else 1)


if __name__ == "__main__":
    main()
