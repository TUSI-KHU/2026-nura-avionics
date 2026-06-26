#!/usr/bin/env python3
import pathlib
import subprocess
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[2]


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="nura_protocol_test_") as build_dir:
        binary = pathlib.Path(build_dir) / "protocol_frame_tests"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{ROOT / 'protocol' / 'include'}",
                str(ROOT / "test" / "protocol" / "protocol_frame_tests.cpp"),
                "-o",
                str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
