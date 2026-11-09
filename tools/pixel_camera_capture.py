"""TYPE8プロジェクトから既存Pixel Cameraを利用する。ロボットのコマンドは送信しない。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

CAMERA_PORT = 8765
BLE_PORT = 8766


def camera_workspace(explicit=None):
    override = explicit or os.environ.get("PIXEL_CAMERA_REPO")
    candidates = [Path(override)] if override else [
        candidate for parent in Path(__file__).resolve().parents
        for candidate in (parent, parent / "Debug_camera")
    ]
    for candidate in candidates:
        candidate = candidate.resolve()
        if (candidate / "python/pixel_camera/client.py").is_file():
            return candidate
    raise RuntimeError("Debug_cameraが見つかりません。--camera-repo または PIXEL_CAMERA_REPO を指定してください")


def camera_python(workspace):
    executable = workspace / ".venv/Scripts/python.exe"
    if not executable.is_file():
        raise RuntimeError("カメラ用Python環境がありません: " + str(executable))
    return executable


def wait_recording(client, recording_id, timeout=12):
    """Acceptance is not readiness. Match recording ID, owner and the RECORDING phase."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.get_status()
        device = status.get("device", {})
        if status.get("connected") and device.get("recording_id") == recording_id:
            if device.get("owner") != client.client_id:
                raise RuntimeError("録画の操作クライアントが一致しません")
            phase = device.get("phase")
            if phase == "RECORDING":
                return device
            if phase in {"IDLE", "ERROR", "STOPPING", "FINALIZING", "TRANSFERRING"}:
                raise RuntimeError("録画中になる前に終了しました: " + phase)
        time.sleep(0.05)
    raise TimeoutError("録画中を確認できません。TYPE8の動作を開始しないでください")


def record_trial(client, project, seconds=30, mouse="unknown", label="", on_ready=None):
    """on_readyは実録画開始後に一度だけ呼ぶ。指定秒数で録画する。"""
    from pixel_camera.transfer import atomic_json
    from pixel_camera.stability import inspect_recording
    if not 1 <= seconds <= 30:
        raise ValueError("録画時間は1〜30秒です")
    folder = Path(project) / "tools/logs/pixel_camera"
    folder.mkdir(parents=True, exist_ok=True)
    manifest = folder / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8] + ".capture.json")
    value = dict(schema_version=1, started_at=datetime.now(timezone.utc).isoformat(),
                 project=str(Path(project).resolve()), state="PREPARING", requested_seconds=seconds,
                 mouse=mouse, mouse_identity_source="user_supplied_not_device_verified", label=label,
                 motion_control="existing_TYPE8_tool_or_explicit_on_ready_callback",
                 type8_log_csv=None, physical_240fps_verified=False)
    atomic_json(manifest, value)
    recording_id = None
    result = None
    try:
        client.connect()
        client.configure(width=1280, height=720, fps=240)
        recording_id = client.start_recording(duration_s=seconds)
        value["recording_id"] = recording_id
        atomic_json(manifest, value)
        ready = wait_recording(client, recording_id)
        value.update(state="RECORDING", ready_at=datetime.now(timezone.utc).isoformat(),
                     ready_elapsed_seconds=ready.get("elapsed_seconds"))
        atomic_json(manifest, value)
        print("RECORDING_READY: " + recording_id + " / TYPE8の既存操作から動作を開始できます", flush=True)
        if on_ready is not None:
            on_ready()
        result = client.wait_for_transfer(timeout=seconds + 120, recording_id=recording_id)
        value.update(video_file=str(result.file_path), metadata_file=str(result.metadata_path),
                     sha256=result.sha256, size_bytes=result.size, source_deleted=result.source_deleted)
        inspection = inspect_recording(result, seconds)
        value.update(state="PASSED" if inspection["passed"] else "FAILED", video_inspection=inspection)
    except (Exception, KeyboardInterrupt) as error:
        value.update(state="INTERRUPTED" if isinstance(error, KeyboardInterrupt) else "FAILED",
                     error=type(error).__name__ + ": " + str(error))
        # A failed/missing start response has unknown outcome: never blindly resend start.
        if recording_id is not None and result is None:
            try:
                status = client.get_status().get("device", {})
                if status.get("recording_id") == recording_id and status.get("owner") == client.client_id \
                        and status.get("phase") in {"PREPARING", "RECORDING"}:
                    client.stop_recording()
                recovered = client.wait_for_transfer(timeout=150, recording_id=recording_id)
                value.update(recovered_video_file=str(recovered.file_path),
                             recovered_source_deleted=recovered.source_deleted)
            except Exception as recovery_error:
                value["recovery_error"] = str(recovery_error)
    finally:
        value["finished_at"] = datetime.now(timezone.utc).isoformat()
        atomic_json(manifest, value)
        client.disconnect()
    return manifest, value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", choices=("gui", "check", "status", "record"), default="gui")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--camera-repo", type=Path)
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--mouse", choices=("A", "B", "unknown"), default="unknown")
    parser.add_argument("--label", default="")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error("--seconds must be between 1 and 30")
    workspace = camera_workspace(args.camera_repo)
    python = camera_python(workspace)
    project = args.project.resolve()
    if not (project / "tools/ble/mouse_gui_ble.py").is_file():
        parser.error("--project must point to the mouse_type8 project")
    if args.action == "check":
        print(json.dumps(dict(project=str(project), camera_repo=str(workspace), camera_python=str(python),
                              camera_port=CAMERA_PORT, ble_gateway_port=BLE_PORT,
                              sends_robot_commands=False), ensure_ascii=False, indent=2))
        return 0
    if args.action == "gui":
        # Use the camera venv so TYPE8's Python/NumPy/Bleak environment is preserved.
        executable = python.with_name("pythonw.exe") if os.name == "nt" else python
        subprocess.Popen([str(executable), "-m", "pixel_camera.gui"], cwd=str(workspace),
                         creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        return 0
    if os.path.normcase(str(Path(sys.executable).resolve())) != os.path.normcase(str(python.resolve())):
        return subprocess.call([str(python), str(Path(__file__).resolve()), *sys.argv[1:]],
                               env=dict(os.environ, PYTHONUTF8="1"))
    from pixel_camera import PixelCameraClient
    client = PixelCameraClient(connection="usb", port=CAMERA_PORT, auto_start=args.action == "record")
    if args.action == "status":
        print(json.dumps(client.get_status(), ensure_ascii=False, indent=2))
        return 0
    manifest, value = record_trial(client, project, args.seconds, args.mouse, args.label)
    print(value["state"] + ": " + str(manifest), flush=True)
    if value.get("video_file"):
        print(value["video_file"], flush=True)
    if value.get("error"):
        print(value["error"], file=sys.stderr)
    return 0 if value["state"] == "PASSED" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
