#!/usr/bin/env python3
"""Build and run host-side flight-state-machine replay tests."""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = ROOT / ".pio" / "build" / "fsm_replay"
BINARY = BUILD_DIR / "fsm_replay_test"
BENCH_BINARY = BUILD_DIR / "fsm_replay_bench_test"


def build_and_run(binary: Path, extra_flags: list[str] | None = None) -> None:
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
        str(binary),
    ]
    if extra_flags:
        cmd[6:6] = extra_flags
    subprocess.run(cmd, cwd=ROOT, check=True)
    subprocess.run([str(binary)], cwd=ROOT, check=True)


def main() -> int:
    build_and_run(BINARY)
    build_and_run(BENCH_BINARY, ["-DNURA_BENCH_FSM_AUTOFLOW=1", "-Wno-unused-function"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
