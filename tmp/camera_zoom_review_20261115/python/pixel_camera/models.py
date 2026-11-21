from dataclasses import dataclass
from pathlib import Path


class PixelCameraError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(f"{code}: {message}")


@dataclass(frozen=True)
class TransferResult:
    recording_id: str
    file_path: Path
    metadata_path: Path
    sha256: str
    size: int
    frame_count: int | None
    frame_count_source: str
    source_deleted: bool
    physical_240fps_verified: bool = False
