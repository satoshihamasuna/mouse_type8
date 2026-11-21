"""Authorized real-device integration run: duplicate prevention, ownership, stop, transfer and deletion."""
from __future__ import annotations

import argparse
import json
import time
import uuid

from pixel_camera import PixelCameraClient, PixelCameraError


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-only", action="store_true", help="Start a 10-second recording; leave it running for a disconnect/recovery test")
    parser.add_argument("--wait-id", help="Wait for a previous recording to transfer after reconnect")
    parser.add_argument("--manual-stop", action="store_true")
    args = parser.parse_args()
    camera = PixelCameraClient(auto_start=False)
    camera.connect()
    if args.wait_id:
        result = camera.wait_for_transfer(recording_id=args.wait_id, timeout=120)
        print(json.dumps({"file": str(result.file_path), "source_deleted": result.source_deleted,
                          "frames": result.frame_count, "recording_id": result.recording_id}), flush=True)
        return
    capabilities = camera.get_capabilities()
    print("DEVICE", capabilities["device"]["model"], capabilities["device"]["android_release"], flush=True)
    camera.configure()
    command_id = uuid.uuid4().hex
    recording_id = camera.start_recording(duration_s=10, command_id=command_id)
    repeated = camera.start_recording(duration_s=10, command_id=command_id)
    assert recording_id == repeated, "Duplicate command started another recording"
    print("STARTED", recording_id, "duplicate replay returned same recording", flush=True)
    peer = PixelCameraClient(auto_start=False)
    for operation in ("start_recording", "stop_recording"):
        try:
            peer.command(operation, {"duration_seconds": 10} if operation == "start_recording" else {})
        except PixelCameraError as error:
            assert error.code in {"BUSY", "NOT_RECORDING_OWNER"}, error
            print("COMPETING_CLIENT_REJECTED", operation, error.code, flush=True)
        else:
            raise AssertionError("Competing client was permitted to control this recording")
    if args.start_only:
        return
    if args.manual_stop:
        time.sleep(3)
        accepted = camera.stop_recording()
        assert accepted["file_finalized"] is False
        print("STOP_ACCEPTED_BEFORE_FINALIZATION", flush=True)
    result = camera.wait_for_transfer(timeout=120)
    assert result.source_deleted
    assert result.file_path.exists() and result.metadata_path.exists()
    print(json.dumps({"file": str(result.file_path), "source_deleted": result.source_deleted,
                      "frames": result.frame_count, "recording_id": result.recording_id}), flush=True)


if __name__ == "__main__":
    main()
