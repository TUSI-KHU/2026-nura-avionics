#!/usr/bin/env python3
"""Build and run host-side flight logging tests."""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = ROOT / ".pio" / "build" / "logging_tests"
BINARY = BUILD_DIR / "logging_tests"


def main() -> int:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Isrc",
        "-Iinclude",
        "src/logging/flight_log_flash_format.cpp",
        "src/logging/flight_log_ram_buffer.cpp",
        "src/logging/flight_log_record.cpp",
        "test/logging/logging_tests.cpp",
        "-o",
        str(BINARY),
    ]
    subprocess.run(cmd, cwd=ROOT, check=True)
    subprocess.run([str(BINARY)], cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
