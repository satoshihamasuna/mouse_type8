"""Durable receive store. A committed receipt is the only authority for source deletion."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import time
import uuid

ID = re.compile(r"^[A-Za-z0-9_-]{1,96}$")
SHA = re.compile(r"^[0-9a-f]{64}$")
MAX_VIDEO = 2 * 1024**3


def digest_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, data: dict) -> None:
    atomic_bytes(path, json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8"))


class ReceiveStore:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.pending = self.root / ".pending"
        self.receipts = self.root / ".receipts"
        self.pending.mkdir(exist_ok=True)
        self.receipts.mkdir(exist_ok=True)

    def path(self, name: str) -> Path:
        candidate = self.root / name
        if candidate.resolve().parent != self.root:
            raise ValueError("PATH_OUTSIDE_SAVE_DIRECTORY")
        return candidate

    def validate(self, recording_id: str, envelope: dict) -> dict:
        if not ID.fullmatch(recording_id):
            raise ValueError("INVALID_RECORDING_ID")
        metadata = envelope.get("metadata", {})
        if metadata.get("recording_id") != recording_id or metadata.get("video_file") != recording_id + ".mp4":
            raise ValueError("RECORDING_ID_MISMATCH")
        if metadata.get("state") != "FINALIZED" or metadata.get("transfer_protocol") != 1:
            raise ValueError("VIDEO_NOT_FINALIZED")
        size = metadata.get("video_size_bytes")
        if type(size) is not int or not 0 < size <= MAX_VIDEO or not SHA.fullmatch(str(metadata.get("sha256", ""))):
            raise ValueError("INVALID_SIZE_OR_HASH")
        sidecars = envelope.get("sidecars", {})
        expected = {metadata.get(key) for key in ("sensor_csv", "started_csv", "pts_csv")}
        if None in expected or set(sidecars) != expected:
            raise ValueError("SIDECAR_SET_MISMATCH")
        for name, content in sidecars.items():
            if name not in {recording_id + suffix for suffix in (".sensor.csv", ".started.csv", ".pts.csv")}:
                raise ValueError("INVALID_SIDECAR_NAME")
            if not isinstance(content, str) or len(content.encode("utf-8")) > 2 * 1024**2:
                raise ValueError("INVALID_SIDECAR_CONTENT")
            self.path(name)
        self.path(recording_id + ".mp4")
        return metadata

    def manifest(self, recording_id: str) -> dict:
        if not ID.fullmatch(recording_id):
            raise ValueError("INVALID_RECORDING_ID")
        envelope = json.loads((self.pending / (recording_id + ".json")).read_text(encoding="utf-8"))
        self.validate(recording_id, envelope)
        return envelope

    @staticmethod
    def identity(metadata: dict) -> tuple:
        return metadata["recording_id"], metadata["video_size_bytes"], metadata["sha256"]

    def prepare(self, recording_id: str, envelope: dict) -> dict:
        metadata = self.validate(recording_id, envelope)
        manifest_path = self.pending / (recording_id + ".json")
        if manifest_path.exists():
            previous = self.manifest(recording_id)
            if self.identity(previous["metadata"]) != self.identity(metadata):
                raise ValueError("EXISTING_RECORDING_CONFLICT")
            envelope = previous
            metadata = previous["metadata"]
        else:
            atomic_json(manifest_path, envelope)
        final = self.path(recording_id + ".mp4")
        if final.exists():
            # An old ACK alone is insufficient: recheck the durable file before sending a new ACK.
            if final.stat().st_size != metadata["video_size_bytes"] or digest_file(final) != metadata["sha256"]:
                raise ValueError("EXISTING_VIDEO_CONFLICT")
            self._write_sidecars(recording_id, envelope)
            receipt = self._commit_receipt(recording_id, metadata)
            return dict(receipt, committed=True)
        return {"recording_id": recording_id, "committed": False}

    def _write_sidecars(self, recording_id: str, envelope: dict) -> None:
        for name, content in envelope["sidecars"].items():
            path = self.path(name)
            data = content.encode("utf-8")
            if path.exists() and path.read_bytes() != data:
                raise ValueError("EXISTING_SIDECAR_CONFLICT")
            if not path.exists():
                atomic_bytes(path, data)
        metadata_path = self.path(recording_id + ".json")
        if metadata_path.exists():
            existing = json.loads(metadata_path.read_text(encoding="utf-8"))
            if self.identity(existing) != self.identity(envelope["metadata"]):
                raise ValueError("EXISTING_METADATA_CONFLICT")
        else:
            metadata = dict(envelope["metadata"], pc_transfer_result="VERIFIED", pc_received_at=time.time())
            atomic_json(metadata_path, metadata)

    def complete(self, recording_id: str, temporary: Path, received: int, actual_sha256: str) -> dict:
        envelope = self.manifest(recording_id)
        metadata = envelope["metadata"]
        if received != metadata["video_size_bytes"] or actual_sha256 != metadata["sha256"]:
            raise ValueError("SIZE_OR_SHA256_MISMATCH")
        if temporary.stat().st_size != received:
            raise ValueError("TEMPORARY_FILE_SIZE_MISMATCH")
        self._write_sidecars(recording_id, envelope)
        final = self.path(recording_id + ".mp4")
        if final.exists():
            if final.stat().st_size != received or digest_file(final) != actual_sha256:
                raise ValueError("EXISTING_VIDEO_CONFLICT")
            temporary.unlink(missing_ok=True)
        else:
            # The receiver fsyncs temporary before calling complete.
            os.replace(temporary, final)
        return dict(self._commit_receipt(recording_id, metadata), committed=True)

    def _commit_receipt(self, recording_id: str, metadata: dict) -> dict:
        receipt_path = self.receipts / (recording_id + ".json")
        old = json.loads(receipt_path.read_text(encoding="utf-8")) if receipt_path.exists() else {}
        receipt = {
            "recording_id": recording_id, "size": metadata["video_size_bytes"], "sha256": metadata["sha256"],
            "file_path": str(self.path(recording_id + ".mp4")), "metadata_path": str(self.path(recording_id + ".json")),
            "frame_count": metadata.get("encoded_timing", {}).get("count"),
            "frame_count_source": "android_encoder_samples", "physical_240fps_verified": False,
            "source_deleted": old.get("source_deleted", False), "received_at": old.get("received_at", time.time()),
        }
        atomic_json(receipt_path, receipt)
        return receipt

    def mark_source_deleted(self, recording_id: str, sha256: str) -> None:
        receipt = self.get(recording_id)
        if receipt and receipt["sha256"] == sha256:
            if receipt.get("source_deleted") is True:
                return  # Replayed acknowledgements must not stall camera commands.
            receipt["source_deleted"] = True
            atomic_json(self.receipts / (recording_id + ".json"), receipt)

    def get(self, recording_id: str) -> dict | None:
        if not ID.fullmatch(recording_id):
            raise ValueError("INVALID_RECORDING_ID")
        path = self.receipts / (recording_id + ".json")
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None

    def history(self) -> list[dict]:
        # The application owns this directory; no user-provided names are evaluated as paths.
        return sorted((json.loads(p.read_text(encoding="utf-8")) for p in self.receipts.glob("*.json")),
                      key=lambda item: item.get("received_at", 0), reverse=True)
