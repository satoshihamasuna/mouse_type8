"""Read-only, frame-accurate access to finalized videos. No nominal-FPS frame synthesis."""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import hashlib
from pathlib import Path

import av


@dataclass(frozen=True)
class FramePoint:
    pts: int
    time_base: Fraction

    @property
    def seconds(self):
        return self.pts * self.time_base


@dataclass(frozen=True)
class VideoIndex:
    path: Path
    size: int
    modified_ns: int
    sha256: str
    width: int
    height: int
    points: tuple[FramePoint, ...]

    @property
    def count(self):
        return len(self.points)

    def relative_seconds(self, index):
        return float(self.points[index].seconds - self.points[0].seconds)

    def interval_ms(self, index):
        return None if index == 0 else float(
            (self.points[index].seconds - self.points[index - 1].seconds) * 1000)

    def ensure_unchanged(self):
        stat = self.path.stat()
        if (stat.st_size, stat.st_mtime_ns) != (self.size, self.modified_ns):
            raise ValueError("動画が解析後に変更されました。開き直してください")

    def evidence(self):
        span = self.relative_seconds(self.count - 1)
        return dict(source_file=str(self.path), source_size=self.size, source_sha256=self.sha256,
                    decoded_frames=self.count, width=self.width, height=self.height,
                    pts_span_seconds=span, fps_from_decoded_pts=(self.count - 1) / span if span > 0 else None,
                    physical_240fps_verified=False,
                    note="Frame PTS and visible changes alone do not prove independent sensor exposures.")


def scan_video(path) -> VideoIndex:
    path = Path(path).resolve()
    if path.name.endswith(".partial.mp4"):
        raise ValueError("未確定のMP4は開けません。録画・転送の完了を待ってください")
    before = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    points = []
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        stream.codec_context.options = {"err_detect": "explode"}
        stream.codec_context.thread_type = "AUTO"
        width, height = stream.codec_context.width, stream.codec_context.height
        for frame in container.decode(stream):
            if frame.is_corrupt or frame.pts is None or frame.time_base is None:
                raise ValueError("破損またはPTSが不明な映像フレームがあります")
            if (frame.width, frame.height) != (width, height):
                raise ValueError("動画の途中で解像度が変わっています")
            point = FramePoint(frame.pts, Fraction(frame.time_base))
            if points and point.seconds <= points[-1].seconds:
                raise ValueError("フレームPTSが単調増加していません")
            points.append(point)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("解析中に動画が変更されました")
    if not points:
        raise ValueError("復号できる映像フレームがありません")
    return VideoIndex(path, after.st_size, after.st_mtime_ns, digest.hexdigest(), width, height, tuple(points))


@dataclass(frozen=True)
class DecodedFrame:
    index: int
    width: int
    height: int
    stride: int
    rgb: bytes


class FrameReader:
    """Single worker owner; sequential steps reuse the decoder, jumps decode from a preceding keyframe."""
    def __init__(self, index: VideoIndex):
        self.index = index
        index.ensure_unchanged()
        self.container = av.open(str(index.path))
        try:
            self.stream = self.container.streams.video[0]
            self.stream.codec_context.options = {"err_detect": "explode"}
            self.iterator = None
            self.previous = None
        except Exception:
            self.container.close()
            raise

    def get(self, position: int) -> DecodedFrame:
        if not 0 <= position < self.index.count:
            raise IndexError("Frame index outside video")
        self.index.ensure_unchanged()
        if self.previous is not None and position == self.previous.index:
            return self.previous
        target = self.index.points[position].seconds
        if self.previous is None or position != self.previous.index + 1:
            self.container.seek(int(target / self.stream.time_base), stream=self.stream, backward=True, any_frame=False)
            self.iterator = self.container.decode(self.stream)
        for frame in self.iterator:
            if frame.pts is None or frame.time_base is None or frame.is_corrupt:
                raise ValueError("フレームを正確に復号できません")
            at = frame.pts * frame.time_base
            if at < target:
                continue
            if at != target:
                raise ValueError("指定フレームのPTSが解析時と一致しません")
            rgb = frame.reformat(format="rgb24")
            plane = rgb.planes[0]
            self.previous = DecodedFrame(position, rgb.width, rgb.height, plane.line_size, bytes(plane))
            return self.previous
        raise ValueError("指定フレームに到達する前に動画が終了しました")

    def close(self):
        self.container.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
