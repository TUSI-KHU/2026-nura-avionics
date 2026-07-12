#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
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
    raise SystemExit("pyserial is missing. Run this with PlatformIO's Python.") from exc


TEENSY_VID_PID = {(0x16C0, 0x0483), (0x16C0, 0x0478)}


def find_teensy_port() -> str | None:
    for port in list_ports.comports():
        if (port.vid, port.pid) in TEENSY_VID_PID or "Teensy" in (port.description or ""):
            return port.device
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Plain Teensy USB serial watcher. No sensor parsing.")
    parser.add_argument("--port", help="Serial port, for example /dev/ttyACM0.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to watch. 0 means forever.")
    args = parser.parse_args()

    port = args.port or find_teensy_port()
    if port is None:
        print("No Teensy serial port found. Check USB cable or press Teensy PROGRAM then upload firmware.", flush=True)
        return 2

    print(f"OPEN {port} baud={args.baud}", flush=True)
    with serial.Serial(port, args.baud, timeout=0.2) as ser:
        ser.dtr = True
        deadline = None if args.duration <= 0 else time.monotonic() + args.duration
        lines = 0
        bytes_seen = 0
        while deadline is None or time.monotonic() < deadline:
            data = ser.readline()
            if not data:
                continue
            lines += 1
            bytes_seen += len(data)
            print(data.decode("utf-8", errors="replace").rstrip("\r\n"), flush=True)

    print(f"CLOSE lines={lines} bytes={bytes_seen}", flush=True)
    return 0 if lines > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
