import asyncio
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer

from pixel_camera.server import Broker
from pixel_camera.transfer import ReceiveStore


def envelope(recording_id="test_record_1", content=b"complete mp4 bytes"):
    metadata = {
        "recording_id": recording_id, "video_file": recording_id + ".mp4", "state": "FINALIZED",
        "transfer_protocol": 1, "video_size_bytes": len(content), "sha256": hashlib.sha256(content).hexdigest(),
        "encoded_timing": {"count": 2400},
        "sensor_csv": recording_id + ".sensor.csv", "started_csv": recording_id + ".started.csv",
        "pts_csv": recording_id + ".pts.csv",
    }
    return {"metadata": metadata, "sidecars": {
        metadata["sensor_csv"]: "frame_number,sensor_timestamp_ns\n7,100\n",
        metadata["started_csv"]: "frame_number,started_timestamp_ns\n7,100\n",
        metadata["pts_csv"]: "encoded_sample_index,presentation_time_ns\n0,0\n",
    }}


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.store = ReceiveStore(self.root)
        self.payload = b"complete mp4 bytes"
        self.envelope = envelope(content=self.payload)

    def tearDown(self):
        self.temp.cleanup()

    def complete(self):
        self.store.prepare("test_record_1", self.envelope)
        temporary = self.root / "test_record_1.mp4.part"
        temporary.write_bytes(self.payload)
        return self.store.complete("test_record_1", temporary, len(self.payload), hashlib.sha256(self.payload).hexdigest())

    def test_commit_persists_video_sidecars_and_receipt_before_ack(self):
        receipt = self.complete()
        self.assertTrue(receipt["committed"])
        self.assertEqual(self.payload, Path(receipt["file_path"]).read_bytes())
        for name in self.envelope["sidecars"]:
            self.assertTrue((self.root / name).exists())
        self.assertTrue((self.root / ".receipts/test_record_1.json").exists())

    def test_hash_mismatch_never_creates_success_receipt(self):
        self.store.prepare("test_record_1", self.envelope)
        temporary = self.root / "test_record_1.mp4.part"
        temporary.write_bytes(b"corrupt")
        with self.assertRaisesRegex(ValueError, "SIZE_OR_SHA256"):
            self.store.complete("test_record_1", temporary, 7, hashlib.sha256(b"corrupt").hexdigest())
        self.assertIsNone(self.store.get("test_record_1"))
        self.assertFalse((self.root / "test_record_1.mp4").exists())
        self.assertTrue(temporary.exists())

    def test_lost_ack_retry_and_pc_restart_do_not_duplicate_the_video(self):
        self.complete()
        restarted = ReceiveStore(self.root)
        repeated = restarted.prepare("test_record_1", self.envelope)
        self.assertTrue(repeated["committed"])
        self.assertEqual(1, len(list(self.root.glob("*.mp4"))))

    def test_existing_receipt_does_not_ack_corrupt_pc_file(self):
        self.complete()
        (self.root / "test_record_1.mp4").write_bytes(b"lost file")
        with self.assertRaisesRegex(ValueError, "EXISTING_VIDEO_CONFLICT"):
            self.store.prepare("test_record_1", self.envelope)

    def test_different_recording_with_same_id_is_rejected(self):
        self.complete()
        with self.assertRaisesRegex(ValueError, "CONFLICT"):
            self.store.prepare("test_record_1", envelope(content=b"other bytes"))

    def test_path_injection_is_rejected(self):
        with self.assertRaises(ValueError):
            self.store.prepare("../escape", envelope("../escape"))

    def test_failed_metadata_commit_cannot_ack_or_promote_video(self):
        self.store.prepare("test_record_1", self.envelope)
        temporary = self.root / "test_record_1.mp4.part"
        temporary.write_bytes(self.payload)
        with patch.object(self.store, "_write_sidecars", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.store.complete("test_record_1", temporary, len(self.payload), hashlib.sha256(self.payload).hexdigest())
        self.assertIsNone(self.store.get("test_record_1"))
        self.assertTrue(temporary.exists())
        self.assertFalse((self.root / "test_record_1.mp4").exists())

    def test_conflicting_pc_metadata_cannot_ack_a_new_video(self):
        self.store.prepare("test_record_1", self.envelope)
        wrong = dict(self.envelope["metadata"], recording_id="other")
        (self.root / "test_record_1.json").write_text(json.dumps(wrong), encoding="utf-8")
        temporary = self.root / "test_record_1.mp4.part"
        temporary.write_bytes(self.payload)
        with self.assertRaisesRegex(ValueError, "EXISTING_METADATA_CONFLICT"):
            self.store.complete("test_record_1", temporary, len(self.payload), hashlib.sha256(self.payload).hexdigest())
        self.assertIsNone(self.store.get("test_record_1"))
        self.assertTrue(temporary.exists())
        self.assertFalse((self.root / "test_record_1.mp4").exists())

    def test_deleted_notification_survives_pc_restart(self):
        self.complete()
        self.store.mark_source_deleted("test_record_1", self.envelope["metadata"]["sha256"])
        self.assertTrue(ReceiveStore(self.root).get("test_record_1")["source_deleted"])


class BrokerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.broker = Broker(Path(self.temp.name), "pairing-token")
        self.client = TestClient(TestServer(self.broker.app()), headers={"Authorization": "Bearer pairing-token"})
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    async def test_unauthenticated_command_is_rejected(self):
        response = await self.client.get("/health", headers={"Authorization": "wrong"})
        self.assertEqual(401, response.status)

    async def test_hash_failure_over_http_is_not_an_ack(self):
        value = envelope(content=b"valid")
        response = await self.client.post("/device/manifests/test_record_1", json=value)
        self.assertEqual(200, response.status)
        response = await self.client.put("/device/files/test_record_1", data=b"wrong")
        self.assertEqual(400, response.status)
        self.assertIsNone(self.broker.store.get("test_record_1"))

    async def test_same_command_id_is_forwarded_once(self):
        socket = await self.client.ws_connect("/ws/device")
        await socket.send_json({"type": "hello", "protocol_version": 1, "device_id": "pixel"})
        command = {"protocol_version": 1, "command_id": "command_1", "client_id": "gui",
                   "command": "start_recording", "payload": {"duration_seconds": 10}}
        pending = asyncio.create_task(self.client.post("/api/command", json=command))
        forwarded = await socket.receive_json(timeout=2)
        self.assertEqual("command_1", forwarded["command_id"])
        accepted = {"type": "response", "command_id": "command_1", "ok": True,
                    "result": {"accepted": True, "recording_id": "same_recording"}}
        await socket.send_json(accepted)
        response = await pending
        self.assertEqual(accepted, await response.json())
        response = await self.client.post("/api/command", json=command)
        self.assertEqual(accepted, await response.json())
        self.assertEqual({}, self.broker.pending)
        with self.assertRaises(asyncio.TimeoutError):
            await socket.receive_json(timeout=0.05)
        await socket.close()

    async def test_command_id_cannot_be_reused_by_another_client(self):
        self.broker.replies["same_id"] = ("old fingerprint", {"ok": True})
        response = await self.client.post("/api/command", json={
            "protocol_version": 1, "command_id": "same_id", "client_id": "script",
            "command": "start_recording", "payload": {}})
        self.assertEqual(400, response.status)


if __name__ == "__main__":
    unittest.main()
