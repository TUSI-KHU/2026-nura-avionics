#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    platformio_python = Path.home() / ".local" / "share" / "pipx" / "venvs" / "platformio" / "bin" / "python"
    if platformio_python.exists() and Path(sys.executable).resolve() != platformio_python.resolve():
        os.execv(str(platformio_python), [str(platformio_python), *sys.argv])
    raise SystemExit(
        "pyserial is missing. Run this with PlatformIO's Python, for example:\n"
        "/home/sb/.local/share/pipx/venvs/platformio/bin/python "
        "sensor_test/run_no_lora_bus_smoke.py"
    ) from exc


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "sensor_test" / "no_lora_bus_smoke_test.ino"
BUILD_ROOT = Path("/tmp/nura-no-lora-bus-smoke")
TEENSY_TOOL_DIR = Path.home() / ".platformio" / "packages" / "tool-teensy"
TEENSY_LOADER = TEENSY_TOOL_DIR / "teensy_loader_cli"
BAUD = 115200
TEENSY_VID_PID = {(0x16C0, 0x0483), (0x16C0, 0x0478)}


def now() -> str:
    return dt.datetime.now().strftime("%H:%M:%S")


def log(message: str) -> None:
    print(f"[{now()}] {message}", flush=True)


def require(path: Path, hint: str) -> None:
    if not path.exists():
        raise FileNotFoundError(f"{path} not found. {hint}")


def run_command(cmd: list[str], *, label: str, cwd: Path = ROOT, timeout_s: float | None = None) -> str:
    log(f"{label}: $ {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    lines: list[str] = []
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    while True:
        if deadline is not None and time.monotonic() > deadline:
            proc.kill()
            raise TimeoutError(f"{label} timed out after {timeout_s:.0f}s")

        line = proc.stdout.readline()
        if line:
            clean = line.rstrip("\n")
            lines.append(clean)
            print(f"[{label}] {clean}", flush=True)
            continue

        returncode = proc.poll()
        if returncode is not None:
            rest = proc.stdout.read()
            if rest:
                for clean in rest.splitlines():
                    lines.append(clean)
                    print(f"[{label}] {clean}", flush=True)
            if returncode != 0:
                raise RuntimeError(f"{label} failed with exit code {returncode}")
            return "\n".join(lines)

        time.sleep(0.05)


def teensy_serial_ports() -> list[str]:
    ports: list[str] = []
    for port in list_ports.comports():
        vid_pid = (port.vid, port.pid)
        if vid_pid in TEENSY_VID_PID or "Teensy" in (port.description or ""):
            if port.device.startswith("/dev/tty"):
                ports.append(port.device)
    return sorted(set(ports))


def wait_for_serial_port(timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ports = teensy_serial_ports()
        if ports:
            return ports[0]
        time.sleep(0.25)
    return None


def build_firmware() -> Path:
    require(SKETCH, "The no-LoRa smoke test sketch is missing.")
    if BUILD_ROOT.exists():
        shutil.rmtree(BUILD_ROOT)
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)

    run_command(
        [
            "pio",
            "ci",
            str(SKETCH),
            "--project-conf",
            str(ROOT / "platformio.ini"),
            "-e",
            "build",
            "--lib",
            str(ROOT / "include"),
            "--build-dir",
            str(BUILD_ROOT),
            "--keep-build-dir",
        ],
        label="build",
    )

    hex_path = BUILD_ROOT / ".pio" / "build" / "build" / "firmware.hex"
    require(hex_path, "PlatformIO did not produce firmware.hex.")
    log(f"firmware ready: {hex_path}")
    return hex_path


def request_bootloader(port: str) -> None:
    log(f"requesting Teensy bootloader from {port} using 134 baud")
    try:
        with serial.Serial(port, 134, timeout=0.2) as ser:
            ser.dtr = False
            time.sleep(0.05)
            ser.dtr = True
            time.sleep(0.05)
    except Exception as exc:
        log(f"134-baud reboot failed on {port}: {exc}")


def upload_firmware(hex_path: Path, port: str | None, upload_timeout_s: float) -> None:
    require(TEENSY_LOADER, "Install PlatformIO Teensy tools first.")
    if port is not None:
        request_bootloader(port)
    else:
        log("no Teensy serial port found before upload")

    log("If upload waits here, press the Teensy PROGRAM button once.")
    run_command(
        [str(TEENSY_LOADER), "-mmcu=TEENSY41", "-w", "-v", str(hex_path)],
        label="upload",
        timeout_s=upload_timeout_s,
    )


def parse_result(line: str, state: dict[str, str]) -> None:
    if line.startswith("RUN "):
        state.clear()
        state["RUN"] = line.split(" ", 1)[1]
    elif line.startswith("I2C_FOUND "):
        found = state.get("I2C_FOUND", "")
        state["I2C_FOUND"] = (found + " " + line.split(" ", 1)[1]).strip()
    elif line.startswith("PASS: ") or line.startswith("FAIL: "):
        status, name = line.split(": ", 1)
        state[name] = status
    elif line.startswith("GPS_UART "):
        state["GPS_UART"] = line.removeprefix("GPS_UART ")
    elif line.startswith("SUMMARY "):
        state["SUMMARY"] = line.split(" ", 1)[1]
        print_summary(state)


def print_summary(state: dict[str, str]) -> None:
    run = state.get("RUN", "?")
    found = state.get("I2C_FOUND", "<none>")
    summary = state.get("SUMMARY", "?")
    log(f"run {run} summary={summary} i2c={found}")
    for name in [
        "LIS3MDL I2C detected",
        "MPL3115A2 I2C detected",
        "LSM6DSO32 SPI detected",
        "H3LIS331DL SPI detected",
        "GY-GPS6MV2 UART NMEA detected",
    ]:
        value = state.get(name, "MISSING")
        print(f"[summary] {value:7s} {name}", flush=True)
    if "GPS_UART" in state:
        print(f"[summary] GPS_UART {state['GPS_UART']}", flush=True)


def monitor(port: str, duration_s: float) -> int:
    log(f"opening monitor on {port} at {BAUD} baud for {duration_s:.0f}s")
    state: dict[str, str] = {}
    pass_seen = False
    fail_seen = False
    line_count = 0

    with serial.Serial(port, BAUD, timeout=0.2) as ser:
        ser.dtr = True
        start = time.monotonic()
        while time.monotonic() - start < duration_s:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            line_count += 1
            print(f"[serial] {line}", flush=True)
            parse_result(line, state)
            if line == "SUMMARY PASS":
                pass_seen = True
            elif line == "SUMMARY FAIL":
                fail_seen = True

    if line_count == 0:
        log("monitor saw no serial output")
        return 2
    if pass_seen:
        log("RESULT: PASS seen")
        return 0
    if fail_seen:
        log("RESULT: FAIL seen; check the per-sensor lines above")
        return 1
    log("RESULT: no SUMMARY line seen")
    return 2


def print_wiring_hint() -> None:
    print(
        """
No-LoRa smoke test wiring:
  I2C  SDA=17 SCL=16  -> LIS3MDL, MPL3115A2
  SPI  MOSI=11 MISO=12 SCK=13
       LSM6DSO32 CS=6
       H3LIS331DL CS=7
  UART GPS TX->15 RX3, GPS RX->14 TX3 baud=9600
  LoRa must stay disconnected for this script.
""".strip(),
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Build, upload, and monitor the no-LoRa bus smoke test.")
    parser.add_argument("--duration", type=float, default=90.0, help="Serial monitor duration in seconds.")
    parser.add_argument("--upload-timeout", type=float, default=40.0, help="Upload wait timeout in seconds.")
    parser.add_argument("--port", help="Force a serial port such as /dev/ttyACM0.")
    parser.add_argument("--no-build", action="store_true", help="Reuse the last fixed build dir firmware.")
    parser.add_argument("--no-upload", action="store_true", help="Skip upload and only monitor.")
    parser.add_argument("--monitor-only", action="store_true", help="Alias for --no-build --no-upload.")
    parser.add_argument("--build-only", action="store_true", help="Build the firmware and exit.")
    args = parser.parse_args()

    if args.monitor_only:
        args.no_build = True
        args.no_upload = True

    print_wiring_hint()

    if args.no_build:
        hex_path = BUILD_ROOT / ".pio" / "build" / "build" / "firmware.hex"
        require(hex_path, "Run without --no-build once first.")
    else:
        hex_path = build_firmware()

    if args.build_only:
        log("build-only requested; exiting before upload")
        return 0

    port = args.port or wait_for_serial_port(2.0)
    if port:
        log(f"Teensy serial detected: {port}")
    else:
        log("Teensy serial not detected yet")

    if not args.no_upload:
        upload_firmware(hex_path, port, args.upload_timeout)
        port = wait_for_serial_port(12.0)
        if port is None:
            log("upload finished, but Teensy serial did not reappear")
            return 2
        log(f"Teensy serial after upload: {port}")
    elif port is None:
        log("monitor requested, but no Teensy serial port is visible")
        return 2

    return monitor(port, args.duration)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("interrupted")
        raise SystemExit(130)
    except Exception as exc:
        log(f"ERROR: {exc}")
        raise SystemExit(2)
