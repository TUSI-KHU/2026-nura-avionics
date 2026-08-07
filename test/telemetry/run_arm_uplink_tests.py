#!/usr/bin/env python3
"""Build and run host-side ARM uplink validation tests."""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="nura_arm_uplink_test_") as build_dir:
        policy_binary = Path(build_dir) / "arm_uplink_policy_tests"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{ROOT / 'src'}",
                f"-I{ROOT / 'include'}",
                f"-I{ROOT / 'protocol' / 'include'}",
                str(ROOT / "test" / "telemetry" / "arm_uplink_policy_tests.cpp"),
                "-o",
                str(policy_binary),
            ],
            check=True,
        )
        subprocess.run([str(policy_binary)], check=True)

        flow_binary = Path(build_dir) / "telemetry_arm_flow_tests"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wno-unused-parameter",
                "-DLOG_LEVEL=1",
                "-DNURA_USE_SX127X_LORA=1",
                f"-I{ROOT / 'test' / 'telemetry' / 'stubs'}",
                f"-I{ROOT / 'src'}",
                f"-I{ROOT / 'include'}",
                f"-I{ROOT / 'protocol' / 'include'}",
                str(ROOT / "src" / "core" / "logger" / "logger.cpp"),
                str(ROOT / "src" / "missions" / "flight" / "flight_trace.cpp"),
                str(ROOT / "src" / "missions" / "flight" / "fsm_task.cpp"),
                str(ROOT / "src" / "missions" / "telemetry" / "telemetry_task.cpp"),
                str(ROOT / "test" / "telemetry" / "telemetry_arm_flow_tests.cpp"),
                "-o",
                str(flow_binary),
            ],
            check=True,
        )
        subprocess.run([str(flow_binary)], check=True)

        fsm_binary = Path(build_dir) / "arm_uplink_fsm_tests"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wno-unused-function",
                "-DLOG_LEVEL=0",
                "-DNURA_LEGACY_TEST_TELEMETRY_STATE=1",
                "-DNURA_ARM_UPLINK_TEST_ONLY=1",
                f"-I{ROOT / 'src'}",
                f"-I{ROOT / 'include'}",
                f"-I{ROOT / 'protocol' / 'include'}",
                str(ROOT / "src" / "core" / "logger" / "logger.cpp"),
                str(ROOT / "src" / "hal" / "mock_flight_data_hal.cpp"),
                str(ROOT / "src" / "missions" / "flight" / "flight_trace.cpp"),
                str(ROOT / "src" / "missions" / "flight" / "fsm_task.cpp"),
                str(ROOT / "test" / "fsm_replay" / "flight_state_machine_replay.cpp"),
                "-o",
                str(fsm_binary),
            ],
            check=True,
        )
        subprocess.run([str(fsm_binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
