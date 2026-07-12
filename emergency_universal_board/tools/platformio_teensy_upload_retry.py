#!/usr/bin/env python3
from __future__ import annotations

import shutil
import subprocess
import sys
import time
from pathlib import Path


def find_loader() -> Path:
    path = shutil.which("teensy_loader_cli")
    if path:
        return Path(path)

    bundled = Path.home() / ".platformio" / "packages" / "tool-teensy" / "teensy_loader_cli"
    if bundled.exists():
        return bundled

    raise FileNotFoundError("teensy_loader_cli not found in PATH or ~/.platformio/packages/tool-teensy")


def run_loader(loader: Path, hex_path: Path, args: list[str]) -> int:
    cmd = [str(loader), "--mcu=TEENSY41", *args, str(hex_path)]
    print("$ " + " ".join(cmd), flush=True)
    proc = subprocess.Popen(cmd)
    return proc.wait()


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: platformio_teensy_upload_retry.py <firmware.hex>", file=sys.stderr)
        return 2

    hex_path = Path(sys.argv[1])
    if not hex_path.exists():
        print(f"firmware not found: {hex_path}", file=sys.stderr)
        return 2

    loader = find_loader()
    attempts = [
        ["-s", "-w", "-v"],
        ["-w", "-v"],
        ["-w", "-v"],
    ]
    for index, args in enumerate(attempts, start=1):
        print(f"teensy upload attempt {index}/{len(attempts)}", flush=True)
        code = run_loader(loader, hex_path, args)
        if code == 0:
            return 0
        print("teensy upload attempt failed; retrying", flush=True)
        time.sleep(0.5)

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
