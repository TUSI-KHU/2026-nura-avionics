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
        "src/logging/flight_log_ram_buffer.cpp",
        "src/logging/flight_log_byte_queue.cpp",
        "src/logging/flight_log_mirror_storage.cpp",
        "src/logging/flight_log_record.cpp",
        "src/missions/flight/flight_trace.cpp",
        "src/core/logger/logger.cpp",
        "src/missions/logging/flight_log_task.cpp",
        "test/logging/logging_tests.cpp",
        "-o",
        str(BINARY),
    ]
    subprocess.run(cmd, cwd=ROOT, check=True)
    subprocess.run([str(BINARY)], cwd=ROOT, check=True)
    subprocess.run(["python3", "test/logging/test_decode_flight_log.py"], cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
