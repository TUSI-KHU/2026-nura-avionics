#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import queue
import re
import shutil
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SENSOR_TEST_DIR = ROOT / "test" / "sensor_test"
if str(SENSOR_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_TEST_DIR))

try:
    import run_teensy_lora_pair as teensy_pair
except ImportError as exc:
    raise SystemExit(f"Could not import shared Teensy upload helpers: {exc}") from exc


SENDER_DIR = ROOT / "sender"
RECEIVER_DIR = ROOT / "receiver"
BAUD = 115200
PIO_FALLBACK = Path.home() / ".platformio" / "penv" / ("Scripts" if os.name == "nt" else "bin") / ("pio.exe" if os.name == "nt" else "pio")
PIO = shutil.which("pio") or str(PIO_FALLBACK)


def log(message: str) -> None:
    teensy_pair.log(message)


def build_project(project_dir: Path, label: str) -> Path:
    teensy_pair.run_command([PIO, "run", "-d", str(project_dir)], label=f"build-{label}", cwd=ROOT)
    hex_path = project_dir / ".pio" / "build" / "teensy41" / "firmware.hex"
    teensy_pair.require_file(hex_path, f"{label} build did not produce firmware.hex.")
    log(f"{label} hex ready: {hex_path}")
    return hex_path


def serial_reader(name: str, port: str, out: queue.Queue[tuple[str, str]]) -> None:
    try:
        with teensy_pair.serial.Serial(port, BAUD, timeout=0.2) as ser:
            ser.reset_input_buffer()
            partial = bytearray()
            while True:
                chunk = ser.read(256)
                if not chunk:
                    continue
                partial.extend(chunk)
                while b"\n" in partial:
                    line, _, rest = partial.partition(b"\n")
                    partial = bytearray(rest)
                    out.put((name, line.rstrip(b"\r").decode("utf-8", errors="replace")))
    except Exception as exc:
        out.put((name, f"MONITOR_ERROR: {exc}"))


def monitor_pair(sender_port, receiver_port, *, duration_s: float, min_fast: int, min_gps: int) -> bool:
    log(
        f"monitor opening SENDER={sender_port.device} RECEIVER={receiver_port.device} "
        f"baud={BAUD} duration={duration_s:.1f}s"
    )
    events: queue.Queue[tuple[str, str]] = queue.Queue()
    threads = [
        threading.Thread(target=serial_reader, args=("SENDER", sender_port.device, events), daemon=True),
        threading.Thread(target=serial_reader, args=("RECEIVER", receiver_port.device, events), daemon=True),
    ]
    for thread in threads:
        thread.start()

    fast_rx = 0
    gps_rx = 0
    force_executed = False
    deprecated_rejected = False
    pair_complete = False
    failures: list[str] = []

    fast_re = re.compile(r"^rx type=FAST\b")
    gps_re = re.compile(r"^rx type=GPS\b")
    control_ack_re = re.compile(
        r"^rx type=CONTROL subtype=ACK command=(?P<command>\S+) .* "
        r"stage=(?P<stage>\S+) result=(?P<result>\S+) reason=(?P<reason>\S+)"
    )

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            name, line = events.get(timeout=0.25)
        except queue.Empty:
            continue

        device = sender_port.device if name == "SENDER" else receiver_port.device
        print(f"[{teensy_pair.now()}][{name} {device}] {line}", flush=True)

        if "FAIL:" in line or "MONITOR_ERROR:" in line:
            failures.append(f"{name}: {line}")
        if name == "RECEIVER" and fast_re.search(line):
            fast_rx += 1
        if name == "RECEIVER" and gps_re.search(line):
            gps_rx += 1
        if name == "RECEIVER" and "PASS: v1_lite_pair_test_complete" in line:
            pair_complete = True

        if name == "RECEIVER":
            match = control_ack_re.search(line)
            if match:
                command = match.group("command")
                stage = match.group("stage")
                result = match.group("result")
                reason = match.group("reason")
                if command == "FORCE_DEPLOY" and stage == "EXECUTED" and result == "OK":
                    force_executed = True
                if (
                    command == "ABORT_PROPULSION_DEPRECATED"
                    and stage == "REJECTED"
                    and result == "NOT_SUPPORTED"
                    and reason == "DEPRECATED_COMMAND"
                ):
                    deprecated_rejected = True

    log("monitor summary")
    log(f"  fast_rx={fast_rx} required={min_fast}")
    log(f"  gps_rx={gps_rx} required={min_gps}")
    log(f"  force_deploy_executed={force_executed}")
    log(f"  deprecated_abort_rejected={deprecated_rejected}")
    log(f"  pair_complete_line={pair_complete}")
    if failures:
        log(f"  failures={len(failures)}")
        for failure in failures[:8]:
            log(f"    {failure}")

    passed = (
        fast_rx >= min_fast
        and gps_rx >= min_gps
        and force_executed
        and deprecated_rejected
        and pair_complete
        and not failures
    )
    log("RESULT: PASS - V1 Lite FAST/GPS/CONTROL pair test passed" if passed else "RESULT: FAIL - V1 Lite pair test did not meet criteria")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description="Build, upload, and monitor NURA V1 Lite sender/receiver LoRa test.")
    parser.add_argument("--tx", help="Force sender board by /dev/ttyACM* or teensy_ports /sys/... address.")
    parser.add_argument("--rx", help="Force receiver board by /dev/ttyACM* or teensy_ports /sys/... address.")
    parser.add_argument("--duration", type=float, default=20.0, help="Serial monitor duration in seconds.")
    parser.add_argument("--min-fast", type=int, default=10, help="Minimum FAST packets required at receiver.")
    parser.add_argument("--min-gps", type=int, default=2, help="Minimum GPS packets required at receiver.")
    parser.add_argument("--build-only", action="store_true", help="Only build both projects.")
    parser.add_argument("--no-upload", action="store_true", help="Build and monitor existing firmware without uploading.")
    parser.add_argument("--monitor-only", action="store_true", help="Skip build/upload and only monitor two serial Teensys.")
    args = parser.parse_args()

    log(f"repo={ROOT}")
    log(f"sender_project={SENDER_DIR}")
    log(f"receiver_project={RECEIVER_DIR}")
    teensy_pair.print_ports("initial USB state")

    sender_hex = SENDER_DIR / ".pio" / "build" / "teensy41" / "firmware.hex"
    receiver_hex = RECEIVER_DIR / ".pio" / "build" / "teensy41" / "firmware.hex"

    if not args.monitor_only:
        sender_hex = build_project(SENDER_DIR, "sender")
        receiver_hex = build_project(RECEIVER_DIR, "receiver")

    if args.build_only:
        log("build-only requested; stopping before upload/monitor")
        return 0

    if args.monitor_only or args.no_upload:
        serials = teensy_pair.wait_for_two_serials(timeout_s=5)
        sender_port, receiver_port = serials[0], serials[1]
        if args.tx and args.rx:
            ports = teensy_pair.list_teensy_ports()
            sender_port = teensy_pair.pick_by_device_or_address(ports, args.tx)
            receiver_port = teensy_pair.pick_by_device_or_address(ports, args.rx)
        log(f"monitor assignment: SENDER={sender_port.device} RECEIVER={receiver_port.device}")
    else:
        sender_port, receiver_port = teensy_pair.upload_pair(sender_hex, receiver_hex, args)

    teensy_pair.print_ports("USB state before monitor")
    return 0 if monitor_pair(sender_port, receiver_port, duration_s=args.duration, min_fast=args.min_fast, min_gps=args.min_gps) else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("interrupted")
        raise SystemExit(130)
    except Exception as exc:
        log(f"ERROR: {exc}")
        raise SystemExit(1)
