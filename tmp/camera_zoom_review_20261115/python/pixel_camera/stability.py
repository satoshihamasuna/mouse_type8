"""Sequential USB recordings with independent FFmpeg decoding and durable QA reports."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import html
import io
import json
from pathlib import Path
import shutil
import threading
import time
import uuid

from .client import PixelCameraClient
from .runtime import PROJECT_ROOT
from .transfer import atomic_bytes, atomic_json

PHYSICAL_NOTE = (
    "Decoded frame count and PTS validate the saved video timeline. They do not prove "
    "240 independent sensor exposures per second. A separately timed moving target is still required."
)


def decode_video(path):
    import av
    points = []
    missing_pts = corrupt = wrong_size = 0
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        stream.codec_context.options = {"err_detect": "explode"}
        stream.codec_context.thread_type = "AUTO"
        codec = stream.codec_context.name
        duration = float(stream.duration * stream.time_base) if stream.duration is not None else None
        count = 0
        for frame in container.decode(stream):
            count += 1
            corrupt += int(frame.is_corrupt)
            wrong_size += int((frame.width, frame.height) != (1280, 720))
            if frame.pts is None:
                missing_pts += 1
            else:
                points.append(float(frame.pts * frame.time_base))
    deltas = [b - a for a, b in zip(points, points[1:])]
    span = points[-1] - points[0] if len(points) > 1 else 0
    return {
        "decoder": "PyAV/FFmpeg", "pyav_version": av.__version__, "codec": codec,
        "decoded_frames": count, "duration_seconds": duration, "pts_span_seconds": span,
        "fps_from_decoded_pts": (len(points) - 1) / span if span > 0 else None,
        "first_pts_seconds": points[0] if points else None,
        "max_interval_ms": max(deltas) * 1000 if deltas else None,
        "gaps_over_1_5_frames": sum(delta > 1.5 / 240 for delta in deltas),
        "non_increasing_pts": sum(delta <= 0 for delta in deltas),
        "missing_pts": missing_pts, "corrupt_frames": corrupt, "wrong_size_frames": wrong_size,
    }


def evaluate_recording(result, metadata, decoded, duration_s):
    errors = []
    def require(condition, code):
        if not condition:
            errors.append(code)
    require(metadata.get("state") == "FINALIZED" and not metadata.get("errors"), "ANDROID_RECORDING_ERROR")
    require(metadata.get("stop_reason") == "DURATION", "UNEXPECTED_STOP_REASON")
    window = metadata.get("recording_window", {})
    require(window.get("limit_reached") is True and window.get("origin") == "first_encoded_frame_pts",
            "FIRST_FRAME_DURATION_NOT_CONFIRMED")
    require(metadata.get("capture_failures") == 0 and metadata.get("buffer_losses") == 0, "CAMERA_CAPTURE_LOSS")
    require(metadata.get("eos_received") is True and metadata.get("muxer_finalized") is True, "INCOMPLETE_MP4")
    count = decoded["decoded_frames"]
    require(count > 1 and count == result.frame_count == metadata.get("encoded_timing", {}).get("count")
            == metadata.get("mp4", {}).get("sample_count") == window.get("written_frames"), "FRAME_COUNT_MISMATCH")
    require(decoded["codec"] == "h264" and decoded["wrong_size_frames"] == 0, "FORMAT_MISMATCH")
    require(not any(decoded[key] for key in ("gaps_over_1_5_frames", "non_increasing_pts",
                                             "missing_pts", "corrupt_frames")), "DECODE_OR_PTS_ERROR")
    fps = decoded["fps_from_decoded_pts"]
    require(fps is not None and abs(fps - 240) <= 4.8, "TARGET_FPS_NOT_OBSERVED")
    duration = decoded["duration_seconds"]
    require(duration is not None and abs(duration - duration_s) <= 0.05, "VIDEO_DURATION_MISMATCH")
    require(duration_s - 0.05 <= decoded["pts_span_seconds"] < duration_s + 0.001, "PTS_SPAN_MISMATCH")
    require(decoded["first_pts_seconds"] is not None and abs(decoded["first_pts_seconds"]) < 0.0001,
            "PTS_ORIGIN_MISMATCH")
    require(metadata.get("video_size_bytes") == result.size and metadata.get("sha256") == result.sha256,
            "RECEIPT_IDENTITY_MISMATCH")
    require(metadata.get("pc_transfer_result") == "VERIFIED", "TRANSFER_NOT_VERIFIED")
    require(result.source_deleted is True, "SOURCE_DELETION_NOT_CONFIRMED")
    thermal = metadata.get("thermal_status_after")
    require(thermal is None or thermal < 3, "SEVERE_THERMAL_STATUS")
    return errors


def inspect_recording(result, duration_s):
    metadata = json.loads(result.metadata_path.read_text(encoding="utf-8"))
    digest = hashlib.sha256()
    with result.file_path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    sha_ok = digest.hexdigest() == result.sha256 and result.file_path.stat().st_size == result.size
    decoded = decode_video(result.file_path)
    errors = evaluate_recording(result, metadata, decoded, duration_s)
    if not sha_ok:
        errors.append("PC_RECHECK_SIZE_OR_SHA256_MISMATCH")
    return dict(
        recording_id=result.recording_id, file_path=str(result.file_path), requested_seconds=duration_s,
        size_bytes=result.size, sha256=result.sha256, sha256_rechecked=sha_ok,
        source_deleted=result.source_deleted, physical_240fps_verified=False,
        **decoded, discarded_tail_frames=metadata.get("recording_window", {}).get("discarded_tail_frames"),
        thermal_before=metadata.get("thermal_status_before"), thermal_after=metadata.get("thermal_status_after"),
        battery_temperature_c_before=metadata.get("battery_temperature_c_before"),
        battery_temperature_c_after=metadata.get("battery_temperature_c_after"),
        android_assessment=metadata.get("assessment"), sensor_sampling=metadata.get("sensor_sampling"),
        first_frame_arrival_latency_seconds=metadata.get("recording_window", {}).get("first_frame_arrival_latency_seconds"),
        errors=errors, passed=not errors,
    )


def save_report(directory, report):
    atomic_json(directory / "report.json", report)
    fields = ["index", "recording_id", "passed", "requested_seconds", "duration_seconds",
              "decoded_frames", "fps_from_decoded_pts", "max_interval_ms", "gaps_over_1_5_frames",
              "discarded_tail_frames", "sha256_rechecked", "source_deleted",
              "thermal_before", "thermal_after", "battery_temperature_c_before",
              "battery_temperature_c_after", "phone_available_bytes_after_transfer",
              "capture_through_deletion_seconds", "errors", "file_path"]
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(report["runs"])
    atomic_bytes(directory / "runs.csv", buffer.getvalue().encode("utf-8-sig"))
    esc = lambda value: html.escape(str(value))
    rows = []
    for row in report["runs"]:
        rows.append("<tr>" + "".join("<td>" + esc(row.get(key, "")) + "</td>" for key in
            ("index", "recording_id", "passed", "duration_seconds", "decoded_frames", "fps_from_decoded_pts",
             "gaps_over_1_5_frames", "discarded_tail_frames", "sha256_rechecked", "source_deleted",
             "battery_temperature_c_after", "thermal_after", "errors")) + "</tr>")
    document = """<!doctype html><html lang="ja"><meta charset="utf-8">
<title>Pixel Camera 連続撮影試験</title><style>
body{font:15px system-ui;margin:32px;color:#17212c}table{border-collapse:collapse;font-size:13px}
td,th{border:1px solid #bdc9d3;padding:8px;text-align:left}th{background:#e8eff5}
code{overflow-wrap:anywhere}p{max-width:1000px}td{white-space:nowrap}
</style><h1>Pixel Camera 連続撮影試験</h1>"""
    document += "<p>状態: <b>" + esc(report["state"]) + "</b> / 完了 " + str(len(report["runs"]))
    document += " / 予定 " + str(report["requested_runs"]) + " 回</p>"
    document += "<p>判定は動画の復号・PTS・サイズ/SHA-256・端末動画の削除通知を対象とします。"
    document += "物理的に異なる240コマ/秒の撮影は未検証です。センサー通知は間引かれる場合があります。"
    document += "温度はバッテリー温度であり、カメラやSoCの温度ではありません。</p>"
    document += "<p>開始: " + esc(report["started_at"]) + "<br>エラー: " + esc(report.get("error", "")) + "</p>"
    document += "<table><thead><tr>" + "".join("<th>" + label + "</th>" for label in
        ("回", "録画ID", "合格", "MP4秒", "復号フレーム数", "PTS実測fps", "PTS欠落候補",
         "終了後除外フレーム", "SHA再照合", "端末削除", "電池°C", "熱状態", "エラー")) + "</tr></thead><tbody>"
    document += "".join(rows) + "</tbody></table></html>"
    atomic_bytes(directory / "report.html", document.encode("utf-8"))


def wait_idle(client, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.get_status()
        device = status.get("device", {})
        if status.get("connected") and device.get("phase") == "IDLE":
            return device
        time.sleep(0.25)
    raise TimeoutError("Phone did not return to IDLE; no new recording was started")


def run_stability(client, *, repetitions=20, duration_s=10, report_dir=None, cancel=None, on_progress=print):
    if not 1 <= repetitions <= 1000 or not 1 <= duration_s <= 30:
        raise ValueError("repetitions must be 1..1000 and duration_s 1..30")
    import av  # Fail before taking a video if the optional QA dependency is missing.
    cancel = cancel or threading.Event()
    parent = Path(report_dir) if report_dir else PROJECT_ROOT / "recordings/reports"
    directory = parent / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, state="RUNNING", started_at=datetime.now(timezone.utc).isoformat(),
                  requested_runs=repetitions, requested_seconds=duration_s, runs=[], pyav_version=av.__version__,
                  physical_240fps_verified=False, verification_note=PHYSICAL_NOTE)
    save_report(directory, report)
    on_progress("Report: " + str(directory / "report.html"))
    active_id = None
    try:
        client.configure()
        report["capabilities"] = client.get_capabilities()
        required = int(repetitions * duration_s * 6_000_000 * 1.3 + 256 * 1024**2)
        if shutil.disk_usage(client.save_dir).free < required:
            raise OSError(f"PC storage requires approximately {required / 1024**3:.2f} GiB free")
        for index in range(1, repetitions + 1):
            if cancel.is_set():
                report["state"] = "CANCELLED"
                break
            wait_idle(client)
            started = time.monotonic()
            on_progress(f"[{index}/{repetitions}] recording {duration_s}s")
            active_id = client.start_recording(duration_s=duration_s)
            # Cancellation finishes the current recording and transfer, then stops before the next clip.
            result = client.wait_for_transfer(timeout=duration_s + 120, recording_id=active_id)
            active_id = None
            capture_seconds = time.monotonic() - started
            device = wait_idle(client)
            on_progress(f"[{index}/{repetitions}] decoding and checking SHA-256")
            row = inspect_recording(result, duration_s)
            row.update(index=index, capture_through_deletion_seconds=capture_seconds,
                       phone_available_bytes_after_transfer=device.get("available_bytes"))
            report["runs"].append(row)
            save_report(directory, report)
            on_progress(f"[{index}/{repetitions}] {'PASS' if row['passed'] else 'FAIL'} "
                        f"frames={row['decoded_frames']} fps={row['fps_from_decoded_pts']} "
                        f"duration={row['duration_seconds']} errors={row['errors']}")
            if not row["passed"]:
                report["state"] = "FAILED"
                break
        else:
            report["state"] = "PASSED"
        if cancel.is_set() and report["state"] == "RUNNING":
            report["state"] = "CANCELLED"
    except (Exception, KeyboardInterrupt) as error:
        report["state"] = "CANCELLED" if isinstance(error, KeyboardInterrupt) else "FAILED"
        report["error"] = f"{type(error).__name__}: {error}"
        # An accepted recording remains identifiable even if the connection/transfer failed.
        if active_id:
            report["unfinished_recording_id"] = active_id
            if isinstance(error, KeyboardInterrupt):
                try:
                    client.stop_recording()
                    client.wait_for_transfer(timeout=150, recording_id=active_id)
                except Exception as finish_error:
                    report["finish_error"] = str(finish_error)
        on_progress(report["error"])
    finally:
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        report["passed_runs"] = sum(row["passed"] for row in report["runs"])
        save_report(directory, report)
    return directory / "report.html", report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--save-dir", type=Path)
    parser.add_argument("--report-dir", type=Path)
    parser.add_argument("--serial")
    args = parser.parse_args()
    client = PixelCameraClient(save_dir=args.save_dir, serial=args.serial)
    client.connect()
    try:
        path, report = run_stability(client, repetitions=args.count, duration_s=args.duration,
                                    report_dir=args.report_dir, on_progress=lambda line: print(line, flush=True))
        print(report["state"] + " — " + str(path), flush=True)
    finally:
        client.disconnect()
    raise SystemExit(0 if report["state"] == "PASSED" else 1)


if __name__ == "__main__":
    main()
