from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid

from .models import PixelCameraError, TransferResult
from .runtime import DEFAULT_RUNTIME, PROJECT_ROOT, config


class PixelCameraClient:
    def __init__(self, connection="usb", save_dir=None, *, adb=None, serial=None,
                 port=8765, host="127.0.0.1", runtime_dir=None, auto_start=True, timeout=25):
        if connection not in {"usb", "wifi"}:
            raise ValueError("connection must be usb or wifi")
        self.connection = connection
        self.save_dir = Path(save_dir).resolve() if save_dir is not None else None
        self.adb = adb or shutil.which("adb") or str(Path(os.environ.get("LOCALAPPDATA", "")) / "Android/Sdk/platform-tools/adb.exe")
        self.serial = serial
        self.port = port
        self.base_url = f"http://{host}:{port}"
        self.runtime_dir = Path(runtime_dir) if runtime_dir else DEFAULT_RUNTIME
        self.auto_start = auto_start
        self.timeout = timeout
        self.client_id = uuid.uuid4().hex
        self.last_recording_id = None
        self.connected = False
        self.token = config(self.runtime_dir, self.port)["token"]

    @property
    def pairing_code(self):
        return self.token

    def _request(self, path, data=None, timeout=None):
        body = None if data is None else json.dumps(data).encode("utf-8")
        request = urllib.request.Request(self.base_url + path, data=body,
                                         headers={"Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                return json.load(response)
        except urllib.error.HTTPError as error:
            try:
                detail = json.loads(error.read())
            except ValueError:
                detail = {"error": str(error)}
            raise PixelCameraError(str(detail.get("error", "HTTP_ERROR")), str(detail)) from error
        except (urllib.error.URLError, TimeoutError, ConnectionError) as error:
            raise PixelCameraError("CONNECTION_FAILED", str(error)) from error

    def ensure_server(self):
        try:
            health = self._request("/health", timeout=2)
        except PixelCameraError as error:
            if error.code != "CONNECTION_FAILED" or not self.auto_start:
                raise
            value = config(self.runtime_dir, self.port, self.save_dir)
            log = (self.runtime_dir / "server.log").open("ab")
            command = [sys.executable, "-m", "pixel_camera.server", "--runtime", str(self.runtime_dir),
                       "--port", str(self.port), "--save-dir", value["save_dir"]]
            try:
                subprocess.Popen(command, cwd=PROJECT_ROOT, stdout=log, stderr=log,
                                 creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            finally:
                log.close()
            deadline = time.monotonic() + 10
            while True:
                try:
                    health = self._request("/health", timeout=2)
                    break
                except PixelCameraError:
                    if time.monotonic() >= deadline:
                        raise PixelCameraError("SERVER_START_FAILED", "See .runtime/server.log")
                    time.sleep(0.2)
        actual = Path(health["save_dir"]).resolve()
        if self.save_dir is not None and actual != self.save_dir:
            raise PixelCameraError("SAVE_DIRECTORY_CONFLICT", f"Existing server saves to {actual}; stop the idle server before changing it")
        self.save_dir = actual

    def _usb_reverse(self):
        command = [self.adb]
        if self.serial:
            command += ["-s", self.serial]
        else:
            result = subprocess.run(command + ["devices"], capture_output=True, text=True, timeout=10, check=True)
            devices = [line.split()[0] for line in result.stdout.splitlines()[1:] if len(line.split()) >= 2 and line.split()[1] == "device"]
            if len(devices) != 1:
                raise PixelCameraError("ADB_DEVICE_SELECTION", "Connect and authorize one device, or specify serial=")
            self.serial = devices[0]
            command += ["-s", self.serial]
        result = subprocess.run(command + ["reverse", f"tcp:{self.port}", f"tcp:{self.port}"],
                                capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise PixelCameraError("ADB_REVERSE_FAILED", result.stderr or result.stdout)

    def connect(self, wait_for_device=20):
        self.ensure_server()
        if self.connection == "usb":
            self._usb_reverse()
        deadline = time.monotonic() + wait_for_device
        while True:
            if self.get_status().get("connected"):
                self.connected = True
                return self
            if time.monotonic() >= deadline:
                raise PixelCameraError("PHONE_NOT_CONNECTED", "スマホで接続コードを入力し「PC接続を開始」を押してください")
            time.sleep(0.25)

    def command(self, command, payload=None, *, command_id=None):
        value = self._request("/api/command", {
            "protocol_version": 1, "command_id": command_id or uuid.uuid4().hex,
            "client_id": self.client_id, "command": command, "payload": payload or {},
        })
        if not value.get("ok"):
            error = value.get("error", {})
            raise PixelCameraError(error.get("code", "COMMAND_FAILED"), error.get("message", str(error)))
        return value.get("result", {})

    def configure(self, width=1280, height=720, fps=240, **settings):
        return self.command("configure", dict(width=width, height=height, fps=fps, **settings))

    def get_capabilities(self):
        return self.command("get_capabilities")

    def get_status(self):
        value = self._request("/api/status")
        self.connected = bool(value.get("connected"))
        return value

    def is_connected(self):
        try:
            return bool(self.get_status().get("connected"))
        except PixelCameraError:
            self.connected = False
            return False

    def start_recording(self, duration_s=30, *, command_id=None):
        result = self.command("start_recording", {"duration_seconds": duration_s}, command_id=command_id)
        self.last_recording_id = result["recording_id"]
        return self.last_recording_id

    def stop_recording(self):
        # This is a stop acceptance response. File finalization and transfer happen afterward.
        return self.command("stop_recording")

    def get_pending_files(self):
        return self.command("get_pending")

    def wait_for_transfer(self, timeout=120, *, recording_id=None, require_source_deleted=True):
        recording_id = recording_id or self.last_recording_id
        if not recording_id:
            raise PixelCameraError("NO_RECORDING", "Start a recording or provide recording_id")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                value = self._request("/api/recordings/" + recording_id,
                                      timeout=min(self.timeout, max(0.1, deadline - time.monotonic())))
            except PixelCameraError as error:
                if error.code != "CONNECTION_FAILED":
                    raise
                self.connected = False
                time.sleep(min(0.25, max(0, deadline - time.monotonic())))
                continue
            if value.get("committed") and (value.get("source_deleted") or not require_source_deleted):
                return TransferResult(recording_id=recording_id, file_path=Path(value["file_path"]),
                                      metadata_path=Path(value["metadata_path"]), sha256=value["sha256"], size=value["size"],
                                      frame_count=value.get("frame_count"), frame_count_source=value["frame_count_source"],
                                      source_deleted=value.get("source_deleted", False))
            if value.get("error"):
                raise PixelCameraError("RECORDING_FAILED", str(value["error"]))
            time.sleep(0.25)
        raise PixelCameraError("TRANSFER_TIMEOUT", "PC履歴と端末の未転送一覧を確認してください。元動画は照合成功まで保持されます")

    def history(self):
        return self._request("/api/history")

    def shutdown_server(self):
        return self._request("/api/shutdown", {})

    def disconnect(self):
        # Broker continues to serve other clients and finish a transfer.
        self.connected = False
