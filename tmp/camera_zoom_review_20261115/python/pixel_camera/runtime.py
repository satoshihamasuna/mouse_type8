from __future__ import annotations

import json
from pathlib import Path
import secrets
import time
import os

from .transfer import atomic_json

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNTIME = PROJECT_ROOT / ".runtime"


def config(runtime_dir: Path, port: int, save_dir: Path | None = None) -> dict:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    path = runtime_dir / f"broker_{port}.json"
    if path.exists():
        for attempt in range(20):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
                break
            except json.JSONDecodeError:
                if attempt == 19:
                    raise
                time.sleep(0.01)
        if save_dir is not None:
            value["save_dir"] = str(save_dir.resolve())
            atomic_json(path, value)
        return value
    value = {"token": secrets.token_hex(16), "port": port,
             "save_dir": str((save_dir or PROJECT_ROOT / "recordings" / "remote").resolve())}
    # Exclusive creation makes simultaneous clients share one pairing code.
    try:
        with path.open("x", encoding="utf-8") as output:
            json.dump(value, output, ensure_ascii=False, indent=2)
            output.flush()
            os.fsync(output.fileno())
    except FileExistsError:
        return config(runtime_dir, port)
    return value
