"""Pixelの録画準備完了後に、既存BLEシェルの単体ターンを一度だけ実行する。"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone, timedelta
import json
import math
import os
import re
import secrets
from pathlib import Path
import subprocess
import sys
import time
import uuid

from pixel_camera_capture import camera_workspace, camera_python


TURNS = (
    "long_r90", "long_l90", "long_r180", "long_l180", "in_r45", "in_l45",
    "out_r45", "out_l45", "in_r135", "in_l135", "out_r135", "out_l135",
    "r_v90", "l_v90",
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def validate_settings(settings, motion, preset):
    """GUI値ではなく実機応答を検証する。初回は回数1・吸引OFFのみ。"""
    if settings.get("error") or settings.get("motion_name") != motion:
        raise RuntimeError("ターンの実機設定を取得できません")
    if settings.get("preset_speed_mm_s") != preset:
        raise RuntimeError("実機プリセットが要求値と一致しません")
    if not math.isclose(settings.get("parameters", {}).get("velo", -1), preset / 1000.0,
                        abs_tol=0.001):
        raise RuntimeError("実機速度が要求値と一致しません")
    if settings.get("turn_count") != 1 or settings.get("suction", {}).get("enabled") is not False:
        raise RuntimeError("この撮影入口は回数1・吸引OFFに限定しています")


def verify_expected_settings(actual, expected):
    """UIから引き継ぐ設定を全物理値で照合する。取得日時・ログ情報は比較しない。"""
    groups = ("parameters", "pid", "feedforward", "suction")
    keys = ("motion_name", "preset_speed_mm_s", "turn_count", "mouse")
    for key in keys + groups:
        if key not in expected or key not in actual:
            raise RuntimeError("照合用設定に必須項目がありません: " + key)
        if key in groups and (not isinstance(expected[key], dict) or not expected[key]):
            raise RuntimeError("照合用設定が空です: " + key)
        if actual[key] != expected[key]:
            raise RuntimeError("UIから引き継ぐ実機設定が一致しません: " + key)


def read_device_settings(shell, motion, preset, parse, mouse, expected=None):
    # RAM値を使う場合はpreset/set/resetを送らない。GUI切断や再接続での変更も検出する。
    command = f"debug turn {motion} show" if expected is not None else f"debug turn {motion} preset {preset}"
    line = shell.query(command, lambda line: line.startswith("DEBUG_TURN_PARAM_X1000 " + motion + " "))
    settings = parse(line, motion, command)
    validate_settings(settings or {}, motion, preset)
    settings.update(mouse=mouse, mouse_identity_source="user_supplied")
    if expected is not None:
        verify_expected_settings(settings, expected)
    return settings


def validate_recording(status, recording_id, owner, seconds):
    device = status.get("device", {})
    if not recording_id or not owner or not status.get("connected") or device.get("phase") != "RECORDING" or \
            device.get("recording_id") != recording_id or device.get("owner") != owner:
        raise RuntimeError("自分の録画中状態を確認できないため走行しません")
    elapsed = device.get("elapsed_seconds")
    if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or not 0 <= elapsed <= seconds - 10:
        raise RuntimeError("録画の残り時間を確認できないため走行しません")


class MotionOnce:
    """record_trialのon_ready専用。途中送信・応答消失でも再送しない。"""
    def __init__(self, shell, motion, event, recording_check, token=None):
        self.shell, self.motion, self.event = shell, motion, event
        self.recording_check = recording_check
        self.token = token if token is not None else secrets.randbelow(99999999) + 1
        self.attempted = False
        self.execution_write_attempted = False
        self.start_confirmed = False

    def __call__(self):
        if self.attempted:
            raise RuntimeError("実行手順は既に開始済みです。自動再送しません")
        self.attempted = True
        self.recording_check()
        self.shell.check_battery()
        self.recording_check()
        self.shell.send(f"debug camera arm {self.token}")
        self.shell.wait_line(lambda line: line == f"DEBUG_CAMERA_ARMED token:{self.token}", timeout=5)
        self.event("camera_permit_confirmed", token=self.token)
        self.recording_check()
        self.execution_write_attempted = True
        command = f"debug turn {self.motion} exe_camera {self.token}"
        self.event("execution_write_attempt", command=command)
        self.shell.send(command)
        self.event("execution_write_returned")
        self.shell.wait_line(lambda line: line == f"DEBUG_CAMERA_RUN_START token:{self.token}", timeout=5)
        self.start_confirmed = True
        self.event("camera_motion_start_confirmed")
        print("CAMERA_MOTION_STARTED: 撮影専用の単体ターンを1回開始しました", flush=True)


class Shell:
    def __init__(self, connection, output, event, char_delay=0.08):
        self.connection, self.output, self.event = connection, output, event
        self.char_delay = char_delay

    def send(self, command):
        # Never retry a write: a timeout can mean that CR reached the mouse.
        self.event("shell_write_attempt", command=command)
        for value in (command + "\r").encode("ascii"):
            self.connection.write(bytes((value,)))
            self.connection.flush()
            time.sleep(self.char_delay)

    def wait_line(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.connection.readline()
            if not raw:
                continue
            self.output.write(raw)
            self.output.flush()
            line = raw.decode("utf-8", errors="replace").replace("\x00", "").strip()
            if line.startswith("DEBUG_CAMERA_ERROR"):
                raise RuntimeError(line)
            if predicate(line):
                return line
        raise TimeoutError("シェル応答を確認できません。実行コマンドは再送しません")

    def query(self, command, predicate, timeout=8):
        self.connection.reset_input_buffer()
        self.send(command)
        return self.wait_line(predicate, timeout)

    def check_battery(self):
        from myshell_debug import BATTERY_RE
        line = self.query("info battery", lambda line: BATTERY_RE.search(line) is not None)
        match = BATTERY_RE.search(line)
        value, limit = int(match["voltage_mv"]), int(match["limit_mv"])
        self.event("battery", voltage_mv=value, limit_mv=limit, status=match["status"])
        if match["status"] != "OK" or value < limit:
            raise RuntimeError("低電圧のため走行を開始しません")


def read_rows(path):
    """更新中のstatus.jsonは読まない。追記CSVの完成行だけを参照する。"""
    if not path.exists():
        return []
    data = path.read_text(encoding="utf-8")
    if not data.endswith("\n"):
        data = data.rsplit("\n", 1)[0] + "\n"
    return list(csv.DictReader(data.splitlines()))


def save_motion_slice(folder, settings):
    rows = read_rows(folder / "telemetry.csv")
    moving = [i for i, row in enumerate(rows)
              if abs(float(row["ideal.velo"])) > 0.01 or abs(float(row["ideal.rad_velo"])) > 0.01]
    if not moving:
        return dict(full_motion_retained=False, reason="No commanded motion in continuous CSV")
    start, end = moving[0], moving[-1]
    start_ms, end_ms = int(rows[start]["time_ms"]), int(rows[end]["time_ms"])
    # cnt is elapsed MCU time maintained by CsvCapture, including uint32 wrap.
    start_elapsed, end_elapsed = int(rows[start]["cnt"]), int(rows[end]["cnt"])
    selected = [row for row in rows if start_elapsed - 500 <= int(row["cnt"]) <= end_elapsed + 500]
    path = folder / "motion.csv"
    with path.open("x", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(selected)
    result = dict(full_motion_retained=start > 0 and end < len(rows) - 1,
                  motion_start_time_ms=start_ms, motion_end_time_ms=end_ms,
                  motion_rows=len(selected), lost_frames=sum(int(row["lost_frames"]) for row in selected),
                  turn_rows=sum(abs(float(row["ideal.rad_velo"])) > 0.01 for row in selected),
                  timebase="MCU time_ms; do not assume one row equals 1 ms")
    sidecar = dict(settings)
    sidecar["log"] = dict(csv=path.name, source_csv="telemetry.csv", frame_count=len(selected),
                          period_ms=5, time_column="time_ms", **result)
    write_json(path.with_suffix(".settings.json"), sidecar)
    return result


def capture(args, project, workspace):
    import serial
    from myshell_debug import DebugGui
    from pixel_camera import PixelCameraClient
    from pixel_camera_capture import record_trial

    expected = None
    if args.use_device_settings:
        expected = json.loads(args.expected_settings.read_text(encoding="utf-8"))
        validate_settings(expected, args.motion, args.preset)
    folder = args.output.resolve() if args.output else project / "tools/logs/pixel_camera" / (
        datetime.now(timezone(timedelta(hours=9))).strftime("%Y%m%d_%H%M%S")
        + f"_{args.mouse}_{args.motion}_{args.preset}_" + uuid.uuid4().hex[:6])
    folder.mkdir(parents=True, exist_ok=False)
    value = dict(schema_version=1, started_at=utc_now(), state="PREPARING", mouse=args.mouse,
                 mouse_identity_source="user_supplied", motion=args.motion, preset=args.preset,
                 turn_count=1, suction_enabled=False, stream_period_ms=5,
                 execution_resend_count=0, type8_log_csv=str(folder / "telemetry.csv"),
                 type8_settings_json=str(folder / "telemetry.settings.json"),
                 physical_240fps_verified=False,
                 settings_mode="verified_device_ram" if expected is not None else "load_preset",
                 camera_connection_mode="reuse_connected" if args.camera_existing_connection else "usb_setup")
    if expected is not None:
        write_json(folder / "expected.settings.json", expected)
        value["expected_settings_json"] = str(folder / "expected.settings.json")
    write_json(folder / "trial.json", value)
    gateway, connection, motion = None, None, None
    settings, manifest, camera_value = None, None, None

    with (folder / "events.jsonl").open("x", encoding="utf-8") as events, \
            (folder / "gateway.stdout.txt").open("xb") as gateway_out, \
            (folder / "gateway.stderr.txt").open("xb") as gateway_err, \
            (folder / "pc.shell.bin").open("xb") as shell_out:
        def event(name, **fields):
            events.write(json.dumps(dict(at=utc_now(), event=name, **fields), ensure_ascii=False) + "\n")
            events.flush()

        try:
            command = [sys.executable, str(project / "tools/ble/mouse_ble_gateway.py"),
                       "--port", "8766", "--address", args.address,
                       "--capture-output", str(folder / "telemetry.csv"), "--capture-period-ms", "5"]
            gateway = subprocess.Popen(command, cwd=project, stdout=gateway_out, stderr=gateway_err,
                env=dict(os.environ, PYTHONUTF8="1"),
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0)
            deadline = time.monotonic() + 35
            while "Ready: socket://127.0.0.1:8766" not in (folder / "gateway.stdout.txt").read_text(
                    encoding="utf-8", errors="replace"):
                if gateway.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("BLE中継を開始できません。gateway.stderr.txtを確認してください")
                time.sleep(0.1)
            connection = serial.serial_for_url("socket://127.0.0.1:8766", timeout=0.2, write_timeout=3)
            shell = Shell(connection, shell_out, event)
            shell.send("shell")
            time.sleep(0.5)
            shell.check_battery()
            capability = shell.query("debug camera show", lambda line: line.startswith("DEBUG_CAMERA_CAP "))
            if re.fullmatch(r"DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000", capability) is None:
                raise RuntimeError("撮影専用の一回実行に対応するファームウェアが必要です")
            settings = read_device_settings(shell, args.motion, args.preset,
                                            DebugGui._parse_device_settings, args.mouse, expected)
            write_json(folder / "telemetry.settings.json", settings)
            deadline = time.monotonic() + 5
            while not read_rows(folder / "telemetry.csv"):
                if time.monotonic() >= deadline or gateway.poll() is not None:
                    raise RuntimeError("連続CSVを受信できないため走行しません")
                time.sleep(0.1)
            before = read_rows(folder / "telemetry.csv")[-1]
            if abs(float(before["ideal.velo"])) > 0.01 or abs(float(before["ideal.rad_velo"])) > 0.01:
                raise RuntimeError("別の走行指令が動作中のため開始しません")
            event("prepared", sequence=int(before["sequence"]), time_ms=int(before["time_ms"]))
            print("PREPARED: preset/velocity/count/suction/battery/continuous CSV checked", flush=True)
            # 接続済みUSB reverseもHTTPで利用できる。再利用時はADBや接続開始を行わない。
            client = PixelCameraClient(connection="wifi" if args.camera_existing_connection else "usb",
                                       port=8765, auto_start=not args.camera_existing_connection)
            if args.camera_existing_connection:
                status = client.get_status()
                if not status.get("connected") or status.get("device", {}).get("phase") != "IDLE":
                    raise RuntimeError("接続済みカメラがIDLEではないため走行しません")
            def recording_check():
                validate_recording(client.get_status(), client.last_recording_id, client.client_id, args.seconds)
            motion = MotionOnce(shell, args.motion, event, recording_check)
            manifest, camera_value = record_trial(client, project, seconds=args.seconds,
                mouse=args.mouse, label=folder.name, on_ready=motion)
            value.update(capture_json=str(manifest), recording_id=camera_value.get("recording_id"),
                         video_file=camera_value.get("video_file"), camera_state=camera_value["state"])
            value["state"] = "CAPTURED_REVIEW_REQUIRED" if camera_value["state"] == "PASSED" else "FAILED"
            value["video_motion_coverage"] = "not_yet_visually_reviewed"
        except (Exception, KeyboardInterrupt) as error:
            value.update(state="FAILED", error=type(error).__name__ + ": " + str(error))
        finally:
            value.update(ready_callback_invoked=bool(motion and motion.attempted),
                         execution_write_attempted=bool(motion and motion.execution_write_attempted),
                         camera_run_start_confirmed=bool(motion and motion.start_confirmed),
                         trigger_mode="recording_only_without_hand")
            if connection is not None:
                connection.close()
            if gateway is not None:
                try:
                    gateway.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    gateway.terminate()
                    gateway.wait(timeout=5)
                value["gateway_exit_code"] = gateway.returncode

    if settings is not None and (folder / "telemetry.csv").exists():
        quality = save_motion_slice(folder, settings)
        status_path = folder / "telemetry.status.json"
        if status_path.exists():
            # Gateway is stopped; reading cannot race with its atomic replace.
            status = json.loads(status_path.read_text(encoding="utf-8"))
            quality["stream_status"] = status
            if status.get("crc_errors", 0) or status.get("lost", 0):
                value["state"] = "INCOMPLETE_LOG"
        value["log_quality"] = quality
        if quality.get("full_motion_retained"):
            value.update(type8_log_csv=str(folder / "motion.csv"),
                         type8_settings_json=str(folder / "motion.settings.json"))
        else:
            value["state"] = "INCOMPLETE_MOTION_NOT_OBSERVED"
            if motion and motion.execution_write_attempted:
                print("MOTION_NOT_OBSERVED: 実行結果を確認できません。再送せず機体を確認してください", flush=True)
    if value.get("gateway_exit_code") not in (None, 0):
        value["state"] = "FAILED_GATEWAY"
    if manifest is not None:
        camera_value.update(type8_log_csv=value["type8_log_csv"],
                            type8_settings_json=value["type8_settings_json"],
                            type8_trial_json=str(folder / "trial.json"),
                            type8_execution_resend_count=0,
                            type8_log_quality=value.get("log_quality"))
        write_json(manifest, camera_value)
    value["finished_at"] = utc_now()
    write_json(folder / "trial.json", value)
    print(value["state"] + ": " + str(folder / "trial.json"), flush=True)
    return 0 if value["state"] == "CAPTURED_REVIEW_REQUIRED" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mouse", choices=("A", "B"), required=True)
    parser.add_argument("--motion", choices=TURNS, default="long_r180")
    parser.add_argument("--preset", type=int, choices=(300, 500, 700, 1000, 1200, 1400, 1500, 1600, 1800, 2000), default=500)
    parser.add_argument("--address", required=True)
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--camera-repo", type=Path)
    parser.add_argument("--use-device-settings", action="store_true", help="プリセットを読み直さず、GUIで変更した実機RAM値を使う")
    parser.add_argument("--expected-settings", type=Path, help="RAM値と全パラメータを照合する設定JSON")
    parser.add_argument("--camera-existing-connection", action="store_true", help="接続済み・IDLEのカメラを再利用し、ADB設定を省く")
    parser.add_argument("--execute", action="store_true", help="明示したときだけ実機へ実行要求を1回送る")
    args = parser.parse_args()
    if not args.execute:
        parser.error("実機撮影には --execute の明示が必要です")
    if not 20 <= args.seconds <= 30:
        parser.error("--seconds must be between 20 and 30")
    if args.use_device_settings != (args.expected_settings is not None):
        parser.error("--use-device-settings と --expected-settings は一緒に指定してください")
    project = Path(__file__).resolve().parents[1]
    workspace = camera_workspace(args.camera_repo)
    python = camera_python(workspace)
    if Path(sys.executable).resolve() != python.resolve():
        return subprocess.call([str(python), str(Path(__file__).resolve()), *sys.argv[1:]],
                               env=dict(os.environ, PYTHONUTF8="1"))
    sys.path.insert(0, str(project / ".tools" / f"ble_python{sys.version_info.major}{sys.version_info.minor}"))
    sys.path.insert(0, str(workspace / "python"))
    return capture(args, project, workspace)


if __name__ == "__main__":
    raise SystemExit(main())
