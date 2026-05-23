#!/usr/bin/env python3
"""Build and run host-side flight-state-machine replay tests."""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = ROOT / ".pio" / "build" / "fsm_replay"
BINARY = BUILD_DIR / "fsm_replay_test"


def main() -> int:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-DLOG_LEVEL=0",
        "-Isrc",
        "-Iinclude",
        "-Iprotocol/include",
        "src/core/logger/logger.cpp",
        "src/hal/mock_flight_data_hal.cpp",
        "src/missions/fsm_task.cpp",
        "test/fsm_replay/flight_state_machine_replay.cpp",
        "-o",
        str(BINARY),
    ]
    subprocess.run(cmd, cwd=ROOT, check=True)
    subprocess.run([str(BINARY)], cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
