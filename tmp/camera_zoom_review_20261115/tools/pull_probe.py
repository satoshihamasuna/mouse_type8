"""Copy Phase 1 debug artifacts without PowerShell binary redirection or device deletion."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path, PurePosixPath
import subprocess
import tarfile


def pull(adb: str, destination: Path, serial: str | None = None) -> list[Path]:
    destination.mkdir(parents=True, exist_ok=True)
    command = [adb] + (["-s", serial] if serial else [])
    command += ["exec-out", "run-as", "com.example.myapplication", "tar", "-cf", "-", "files/probe"]
    # Stream to disk: a 30-second high-speed recording can be hundreds of MB.
    archive = destination / "probe-export.tar.part"
    with archive.open("wb") as output:
        result = subprocess.run(command, stdout=output, stderr=subprocess.PIPE, timeout=180)
    if result.returncode:
        raise RuntimeError(result.stderr.decode("utf-8", errors="replace") + "\n" +
                           "Install a debug APK, unlock the device and finish the recording before export.")
    copied = []
    with tarfile.open(archive, "r:") as contents:
        for member in contents:
            if member.isdir():
                continue
            path = PurePosixPath(member.name)
            # Accept only flat files from this app's probe directory, never links or arbitrary paths.
            if (not member.isfile() or len(path.parts) != 3 or path.parts[:2] != ("files", "probe")
                    or path.name in (".", "..") or ":" in path.name or "\\" in path.name):
                raise ValueError(f"Unexpected archive member: {member.name}")
            target = destination / path.name
            source = contents.extractfile(member)
            if source is None:
                raise ValueError(f"Unreadable member: {member.name}")
            temporary = target.with_name(target.name + ".copying")
            digest = hashlib.sha256()
            with source, temporary.open("wb") as output:
                while chunk := source.read(1024 * 1024):
                    digest.update(chunk)
                    output.write(chunk)
                output.flush()
            if target.exists():
                existing = _hash(target)
                if digest.hexdigest() == existing:
                    temporary.unlink()
                elif target.name == "capabilities.json":
                    target = destination / f"capabilities_{digest.hexdigest()}.json"
                    if target.exists():
                        if _hash(target) != digest.hexdigest():
                            raise FileExistsError(f"Conflicting snapshot: {target}")
                        temporary.unlink()
                    else:
                        temporary.replace(target)
                else:
                    temporary.unlink()
                    raise FileExistsError(f"Different existing file, preserved both source and destination: {target}")
            else:
                temporary.replace(target)
            copied.append(target)
    archive.unlink()
    return copied


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adb", default="adb")
    parser.add_argument("--serial")
    parser.add_argument("--out", type=Path, default=Path("recordings"))
    args = parser.parse_args()
    for path in pull(args.adb, args.out, args.serial):
        print(path)
    print("Files copied. Nothing was removed from the phone. Run verify_recording.py on each recording JSON.")


if __name__ == "__main__":
    main()
