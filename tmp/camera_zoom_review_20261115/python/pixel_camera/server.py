from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import secrets
import time

from aiohttp import web, WSMsgType

from .runtime import DEFAULT_RUNTIME, config
from .transfer import ReceiveStore, ID


class Broker:
    def __init__(self, save_dir: Path, token: str):
        self.store = ReceiveStore(save_dir)
        self.token = token
        self.device = None
        self.device_id = None
        self.device_status = {}
        self.pending = {}
        self.replies = {}
        self.transfers = {}
        self.upload_locks = {}
        self.active_uploads = 0
        self.shutdown = asyncio.Event()

    def app(self) -> web.Application:
        @web.middleware
        async def guard(request, handler):
            supplied = request.headers.get("Authorization", "")
            if not secrets.compare_digest(supplied, "Bearer " + self.token):
                return web.json_response({"error": "UNAUTHORIZED"}, status=401)
            try:
                return await handler(request)
            except (ValueError, KeyError) as error:
                return web.json_response({"error": str(error)}, status=400)

        app = web.Application(middlewares=[guard], client_max_size=8 * 1024**2)
        app.add_routes([
            web.get("/health", self.health), web.get("/ws/device", self.device_socket),
            web.get("/api/status", self.status), web.post("/api/command", self.command),
            web.get("/api/history", self.history), web.get("/api/recordings/{id}", self.recording),
            web.post("/api/shutdown", self.stop),
            web.post("/device/manifests/{id}", self.manifest), web.put("/device/files/{id}", self.upload),
        ])
        return app

    async def health(self, request):
        return web.json_response({"protocol_version": 1, "save_dir": str(self.store.root)})

    async def status(self, request):
        return web.json_response({"connected": self.device is not None and not self.device.closed and self.device_id is not None,
                                  "device_id": self.device_id, "device": self.device_status,
                                  "transfers": self.transfers, "save_dir": str(self.store.root)})

    async def history(self, request):
        return web.json_response(await asyncio.to_thread(self.store.history))

    async def recording(self, request):
        recording_id = request.match_info["id"]
        receipt = await asyncio.to_thread(self.store.get, recording_id)
        if receipt:
            return web.json_response(dict(receipt, committed=True))
        error = self.device_status.get("last_error") if self.device_status.get("recording_id") == recording_id else None
        return web.json_response({"recording_id": recording_id, "committed": False, "error": error})

    async def device_socket(self, request):
        ws = web.WebSocketResponse(heartbeat=15, max_msg_size=1024**2)
        if self.device is not None and not self.device.closed:
            return web.json_response({"error": "DEVICE_ALREADY_CONNECTED"}, status=409)
        await ws.prepare(request)
        self.device = ws
        self.device_id = None
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                value = json.loads(message.data)
                kind = value.get("type")
                if kind == "hello":
                    if value.get("protocol_version") != 1 or not ID.fullmatch(str(value.get("device_id", ""))):
                        await ws.close(code=1008, message=b"unsupported protocol or device")
                        break
                    self.device_id = value["device_id"]
                    self.device_status = value.get("status", {})
                elif self.device_id is None:
                    await ws.close(code=1008, message=b"hello required")
                    break
                elif kind == "response":
                    command_id = value.get("command_id")
                    pending = self.pending.pop(command_id, None)
                    if pending is not None:
                        future, fingerprint = pending
                        self.replies[command_id] = (fingerprint, value)
                        while len(self.replies) > 256:
                            self.replies.pop(next(iter(self.replies)))
                        if not future.done():
                            future.set_result(value)
                elif kind == "status":
                    self.device_status = value.get("status", {})
                elif kind == "source_deleted":
                    await asyncio.to_thread(self.store.mark_source_deleted, value["recording_id"], value["sha256"])
                elif kind == "recording_finalized":
                    self.device_status.update(last_finalized_id=value.get("recording_id"))
        except (ValueError, KeyError):
            await ws.close(code=1008, message=b"invalid message")
        finally:
            if self.device is ws:
                self.device = None
                for future, _ in self.pending.values():
                    if not future.done():
                        future.set_result({"ok": False, "error": {"code": "DEVICE_DISCONNECTED", "message": "接続が切れました"}})
                self.pending.clear()
        return ws

    async def command(self, request):
        value = await request.json()
        if value.get("protocol_version") != 1:
            raise ValueError("UNSUPPORTED_PROTOCOL")
        command_id = str(value.get("command_id", ""))
        if not ID.fullmatch(command_id) or not ID.fullmatch(str(value.get("client_id", ""))):
            raise ValueError("INVALID_COMMAND_OR_CLIENT_ID")
        if value.get("command") not in {"get_status", "get_capabilities", "configure", "start_recording", "stop_recording", "get_pending"}:
            raise ValueError("UNKNOWN_COMMAND")
        fingerprint = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
        cached = self.replies.get(command_id)
        if cached:
            if cached[0] != fingerprint:
                raise ValueError("COMMAND_ID_CONFLICT")
            return web.json_response(cached[1])
        if self.device is None or self.device.closed or self.device_id is None:
            return web.json_response({"error": "DEVICE_NOT_CONNECTED"}, status=503)
        existing = self.pending.get(command_id)
        if existing:
            if existing[1] != fingerprint:
                raise ValueError("COMMAND_ID_CONFLICT")
            future = existing[0]
        else:
            if len(self.pending) >= 64:
                return web.json_response({"error": "TOO_MANY_PENDING_COMMANDS"}, status=429)
            future = asyncio.get_running_loop().create_future()
            self.pending[command_id] = (future, fingerprint)
            await self.device.send_json(dict(value, type="command"))
        try:
            return web.json_response(await asyncio.wait_for(asyncio.shield(future), timeout=20))
        except asyncio.TimeoutError:
            return web.json_response({"error": "DEVICE_COMMAND_TIMEOUT", "command_id": command_id}, status=504)

    async def manifest(self, request):
        recording_id = request.match_info["id"]
        envelope = await request.json()
        lock = self.upload_locks.setdefault(recording_id, asyncio.Lock())
        async with lock:
            result = await asyncio.to_thread(self.store.prepare, recording_id, envelope)
        return web.json_response(result)

    async def upload(self, request):
        recording_id = request.match_info["id"]
        lock = self.upload_locks.setdefault(recording_id, asyncio.Lock())
        async with lock:
            envelope = await asyncio.to_thread(self.store.manifest, recording_id)
            metadata = envelope["metadata"]
            if request.content_length != metadata["video_size_bytes"]:
                raise ValueError("CONTENT_LENGTH_MISMATCH")
            # Retry an upload whose final ACK was lost without creating another file.
            existing = await asyncio.to_thread(self.store.prepare, recording_id, envelope)
            if existing.get("committed"):
                return web.json_response(existing)
            temporary = self.store.path(recording_id + ".mp4.part")
            digest = hashlib.sha256()
            received = 0
            started = time.monotonic()
            self.active_uploads += 1
            self.transfers[recording_id] = {"state": "RECEIVING", "received": 0, "total": request.content_length}
            try:
                with temporary.open("wb") as output:
                    async for chunk in request.content.iter_chunked(256 * 1024):
                        received += len(chunk)
                        if received > metadata["video_size_bytes"]:
                            raise ValueError("TOO_MANY_BYTES")
                        output.write(chunk)
                        digest.update(chunk)
                        self.transfers[recording_id] = {
                            "state": "RECEIVING", "received": received, "total": request.content_length,
                            "bytes_per_second": received / max(0.001, time.monotonic() - started),
                        }
                    output.flush()
                    await asyncio.to_thread(os.fsync, output.fileno())
                receipt = await asyncio.to_thread(self.store.complete, recording_id, temporary, received, digest.hexdigest())
                self.transfers[recording_id] = {"state": "VERIFIED", "received": received, "total": received,
                                               "bytes_per_second": received / max(0.001, time.monotonic() - started)}
                return web.json_response(receipt)
            except BaseException:
                self.transfers[recording_id] = {"state": "FAILED_RETAIN_SOURCE", "received": received, "total": request.content_length}
                raise
            finally:
                self.active_uploads -= 1

    async def stop(self, request):
        if self.active_uploads or self.device_status.get("phase", "IDLE") not in {"IDLE", "ERROR", "DISCONNECTED"}:
            return web.json_response({"error": "BUSY"}, status=409)
        asyncio.get_running_loop().call_later(0.25, self.shutdown.set)
        return web.json_response({"stopping": True})


async def serve(args):
    value = config(args.runtime, args.port, args.save_dir)
    broker = Broker(Path(value["save_dir"]), value["token"])
    runner = web.AppRunner(broker.app())
    await runner.setup()
    site = web.TCPSite(runner, args.bind, args.port)
    await site.start()
    print(f"Pixel Camera server: {args.bind}:{args.port}", flush=True)
    print(f"Save directory: {value['save_dir']}", flush=True)
    print(f"Phone connection code: {value['token']}", flush=True)
    try:
        await broker.shutdown.wait()
    finally:
        if broker.device is not None:
            await broker.device.close()
        await runner.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save-dir", type=Path)
    parser.add_argument("--runtime", type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--bind", default="127.0.0.1", help="USB defaults to loopback only")
    args = parser.parse_args()
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
