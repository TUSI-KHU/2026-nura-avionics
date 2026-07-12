#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is missing. Run with PlatformIO's Python, for example: "
        "/home/sb/.local/share/pipx/venvs/platformio/bin/python "
        "sensor_test/run_teensy_lora_pair.py"
    ) from exc


ROOT = Path(__file__).resolve().parents[1]
SENSOR_TEST_DIR = ROOT / "sensor_test"
SENDER_SKETCH = SENSOR_TEST_DIR / "sx127x_lora_teensy_sender.ino"
RECEIVER_SKETCH = SENSOR_TEST_DIR / "sx127x_lora_teensy_receiver.ino"
BUILD_ROOT = Path("/tmp/nura-lora-teensy-pair")
TEENSY_TOOL_DIR = Path.home() / ".platformio" / "packages" / "tool-teensy"


def teensy_tool(name: str) -> Path:
    executable = TEENSY_TOOL_DIR / name
    if executable.exists() or os.name != "nt":
        return executable
    return TEENSY_TOOL_DIR / f"{name}.exe"


TEENSY_PORTS = teensy_tool("teensy_ports")
TEENSY_REBOOT = teensy_tool("teensy_reboot")
TEENSY_LOADER = teensy_tool("teensy_loader_cli")
TEENSY_POST_COMPILE = teensy_tool("teensy_post_compile")
LORA_LIB_DIR = ROOT / ".pio" / "libdeps" / "build" / "LoRa"
BAUD = 115200


@dataclasses.dataclass(frozen=True)
class TeensyPort:
    address: str
    device: str
    board: str
    mode: str

    @property
    def is_serial(self) -> bool:
        return self.mode.lower() == "serial" and (
            self.device.startswith("/dev/tty") or self.device.startswith("/dev/cu.")
            or re.match(r"^COM\d+$", self.device, re.IGNORECASE)
        )

    @property
    def is_bootloader(self) -> bool:
        return self.mode.lower() == "bootloader"


def now() -> str:
    return dt.datetime.now().strftime("%H:%M:%S")


def log(message: str) -> None:
    print(f"[{now()}] {message}", flush=True)


def run_command(cmd: list[str], *, label: str, cwd: Path = ROOT, check: bool = True) -> subprocess.CompletedProcess[str]:
    printable = " ".join(cmd)
    log(f"{label}: $ {printable}")
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
    for line in proc.stdout:
        line = line.rstrip("\n")
        lines.append(line)
        print(f"[{label}] {line}", flush=True)
    returncode = proc.wait()
    if check and returncode != 0:
        raise RuntimeError(f"{label} failed with exit code {returncode}")
    return subprocess.CompletedProcess(cmd, returncode, "\n".join(lines), "")


def require_file(path: Path, hint: str) -> None:
    if not path.exists():
        raise FileNotFoundError(f"{path} not found. {hint}")


def parse_teensy_ports(raw: str) -> list[TeensyPort]:
    ports: list[TeensyPort] = []
    pattern = re.compile(r"^(?P<address>(?:/sys/|usb:)\S+)\s+(?P<device>(?:/\S+|COM\d+|\[no_device\]))\s+\((?P<board>[^)]*)\)\s+(?P<mode>\S+)", re.IGNORECASE)
    for line in raw.splitlines():
        match = pattern.match(line.strip())
        if not match:
            continue
        ports.append(
            TeensyPort(
                address=match.group("address"),
                device=match.group("device"),
                board=match.group("board"),
                mode=match.group("mode"),
            )
        )
    return ports


def list_teensy_ports(*, show_raw: bool = False) -> list[TeensyPort]:
    require_file(TEENSY_PORTS, "Install PlatformIO Teensy tools first.")
    completed = subprocess.run(
        [str(TEENSY_PORTS), "-L"],
        cwd=str(ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    raw = completed.stdout.strip()
    if show_raw:
        log("teensy_ports -L raw output:")
        if raw:
            for line in raw.splitlines():
                print(f"[ports] {line}", flush=True)
        else:
            print("[ports] <empty>", flush=True)
    return parse_teensy_ports(raw)


def print_ports(title: str) -> list[TeensyPort]:
    log(title)
    ports = list_teensy_ports(show_raw=True)
    serials = [p for p in ports if p.is_serial]
    bootloaders = [p for p in ports if p.is_bootloader]
    log(f"detected serial={len(serials)} bootloader={len(bootloaders)}")
    for port in ports:
        log(f"  {port.mode:10s} {port.device:22s} {port.board:12s} {port.address}")
    return ports


def wait_for(predicate, *, timeout_s: float, description: str) -> object:
    deadline = time.monotonic() + timeout_s
    last_ports: list[TeensyPort] = []
    while time.monotonic() < deadline:
        ports = list_teensy_ports()
        last_ports = ports
        result = predicate(ports)
        if result:
            return result
        time.sleep(0.25)
    log(f"timeout while waiting for {description}; last port state:")
    for port in last_ports:
        log(f"  {port.mode:10s} {port.device:22s} {port.board:12s} {port.address}")
    raise TimeoutError(description)


def wait_for_address_mode(address: str, mode: str, *, timeout_s: float) -> TeensyPort:
    def find_port(ports: list[TeensyPort]) -> TeensyPort | None:
        for port in ports:
            if port.address == address and port.mode.lower() == mode.lower():
                return port
        return None

    return wait_for(find_port, timeout_s=timeout_s, description=f"{address} to become {mode}")


def get_address_mode(address: str, mode: str) -> TeensyPort | None:
    for port in list_teensy_ports():
        if port.address == address and port.mode.lower() == mode.lower():
            return port
    return None


def wait_for_two_serials(*, timeout_s: float) -> list[TeensyPort]:
    def find_serials(ports: list[TeensyPort]) -> list[TeensyPort] | None:
        serials = sorted([p for p in ports if p.is_serial], key=lambda p: p.address)
        return serials if len(serials) >= 2 else None

    return wait_for(find_serials, timeout_s=timeout_s, description="two Teensy serial ports")


def build_sketch(sketch: Path, build_name: str) -> Path:
    require_file(sketch, "Sketch file is missing.")
    require_file(LORA_LIB_DIR, "Run a normal PlatformIO build once so .pio/libdeps/build/LoRa exists.")
    build_dir = BUILD_ROOT / build_name
    if build_dir.exists():
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    run_command(
        [
            "pio",
            "ci",
            str(sketch),
            "-b",
            "teensy41",
            "--keep-build-dir",
            "--build-dir",
            str(build_dir),
            "-l",
            str(LORA_LIB_DIR),
        ],
        label=f"build-{build_name}",
    )
    hex_path = build_dir / ".pio" / "build" / "teensy41" / "firmware.hex"
    require_file(hex_path, "PlatformIO build did not produce firmware.hex.")
    log(f"{build_name} hex ready: {hex_path}")
    return hex_path


def program_current_bootloader(hex_path: Path, label: str, port: TeensyPort | None = None) -> None:
    if TEENSY_LOADER.exists():
        run_command(
            [str(TEENSY_LOADER), "--mcu=TEENSY41", "-w", "-v", str(hex_path)],
            label=f"upload-{label}",
        )
        return

    require_file(TEENSY_POST_COMPILE, "Install PlatformIO Teensy tools first.")
    command = [
        str(TEENSY_POST_COMPILE),
        f"-file={hex_path.stem}",
        f"-path={hex_path.parent}",
        f"-tools={TEENSY_TOOL_DIR}",
        "-board=TEENSY41",
    ]
    if port is None or not port.is_bootloader:
        command.append("-reboot")
    if port is not None:
        command.extend([
            f"-port={port.address}",
            "-portprotocol=Teensy",
        ])
        if port.device != "[no_device]":
            command.append(f"-portlabel={port.device}")
    run_command(command, label=f"upload-{label}")


def request_bootloader(port: TeensyPort, label: str) -> None:
    require_file(TEENSY_REBOOT, "Install PlatformIO Teensy tools first.")
    command = [
        str(TEENSY_REBOOT),
        f"-port={port.address}",
        f"-portlabel={port.device}",
        "-portprotocol=Teensy",
    ]
    result = run_command(command, label=f"reboot-{label}", check=False)
    if result.returncode == 0:
        return
    if get_address_mode(port.address, "Bootloader"):
        log(f"reboot-{label}: target entered bootloader even though teensy_reboot returned {result.returncode}")
        return

    log(f"reboot-{label}: teensy_reboot returned {result.returncode}; trying 134-baud serial reboot")
    try:
        with serial.Serial(port.device, 134, timeout=0.25) as ser:
            ser.dtr = False
            time.sleep(0.05)
            ser.dtr = True
            time.sleep(0.05)
    except Exception as exc:
        try:
            wait_for_address_mode(port.address, "Bootloader", timeout_s=5)
        except TimeoutError:
            raise RuntimeError(f"could not request bootloader for {label} on {port.device}: {exc}") from exc
        else:
            log(f"reboot-{label}: target entered bootloader during 134-baud attempt")
            return
    wait_for_address_mode(port.address, "Bootloader", timeout_s=5)


def upload_to_serial_port(hex_path: Path, port: TeensyPort, label: str) -> TeensyPort:
    log(f"{label}: target serial before upload: {port.device} at {port.address}")
    request_bootloader(port, label)
    bootloader_port = wait_for_address_mode(port.address, "Bootloader", timeout_s=12)
    log(f"{label}: bootloader visible: {bootloader_port.device} at {bootloader_port.address}")
    program_current_bootloader(hex_path, label, bootloader_port)
    serial_port = wait_for_address_mode(port.address, "Serial", timeout_s=15)
    log(f"{label}: serial after upload: {serial_port.device} at {serial_port.address}")
    return serial_port


def pick_by_device_or_address(ports: list[TeensyPort], token: str) -> TeensyPort:
    for port in ports:
        if port.device == token or port.address == token:
            return port
    raise ValueError(f"Could not find Teensy port/address: {token}")


def upload_pair(sender_hex: Path, receiver_hex: Path, args: argparse.Namespace) -> tuple[TeensyPort, TeensyPort]:
    ports = print_ports("USB state before upload")
    serials = sorted([p for p in ports if p.is_serial], key=lambda p: p.address)
    bootloaders = sorted([p for p in ports if p.is_bootloader], key=lambda p: p.address)

    if args.tx and args.rx:
        tx_target = pick_by_device_or_address(ports, args.tx)
        rx_target = pick_by_device_or_address(ports, args.rx)
        if tx_target.is_bootloader and rx_target.is_bootloader:
            raise RuntimeError("Both forced targets are bootloaders; upload order would be ambiguous.")
        tx_port: TeensyPort | None = None
        rx_port: TeensyPort | None = None

        if tx_target.is_bootloader:
            log(f"TX target is already bootloader: {tx_target.address}")
            program_current_bootloader(sender_hex, "tx", tx_target)
            tx_port = wait_for_address_mode(tx_target.address, "Serial", timeout_s=15)
        if rx_target.is_bootloader:
            log(f"RX target is already bootloader: {rx_target.address}")
            program_current_bootloader(receiver_hex, "rx", rx_target)
            rx_port = wait_for_address_mode(rx_target.address, "Serial", timeout_s=15)

        if not tx_target.is_bootloader:
            fresh_tx = get_address_mode(tx_target.address, "Serial")
            if not fresh_tx:
                raise RuntimeError(f"TX target is not a serial port anymore: {tx_target.address}")
            tx_port = upload_to_serial_port(sender_hex, fresh_tx, "tx")
        if not rx_target.is_bootloader:
            fresh_rx = get_address_mode(rx_target.address, "Serial")
            if not fresh_rx:
                raise RuntimeError(f"RX target is not a serial port anymore: {rx_target.address}")
            rx_port = upload_to_serial_port(receiver_hex, fresh_rx, "rx")

        assert tx_port is not None and rx_port is not None
        return tx_port, rx_port

    if len(bootloaders) == 1 and len(serials) == 1:
        tx_bootloader = bootloaders[0]
        rx_serial = serials[0]
        log("auto assignment: bootloader board -> TX, existing serial board -> RX")
        log(f"TX bootloader address: {tx_bootloader.address}")
        log(f"RX serial port: {rx_serial.device} at {rx_serial.address}")
        program_current_bootloader(sender_hex, "tx", tx_bootloader)
        tx_port = wait_for_address_mode(tx_bootloader.address, "Serial", timeout_s=15)
        rx_port = upload_to_serial_port(receiver_hex, rx_serial, "rx")
        return tx_port, rx_port

    if len(serials) == 2 and len(bootloaders) == 0:
        log("auto assignment: first serial by USB address -> TX, second serial -> RX")
        tx_port = upload_to_serial_port(sender_hex, serials[0], "tx")
        ports_after_tx = list_teensy_ports()
        rx_candidate = pick_by_device_or_address(ports_after_tx, serials[1].address)
        rx_port = upload_to_serial_port(receiver_hex, rx_candidate, "rx")
        return tx_port, rx_port

    raise RuntimeError(
        "Need exactly two Teensy boards. Auto mode supports either "
        "1 bootloader + 1 serial, or 2 serial ports. Use --tx and --rx with "
        "a /dev/ttyACM* path or /sys/... address if you want to force assignment."
    )


def serial_reader(name: str, port: str, out: queue.Queue[tuple[str, str]]) -> None:
    try:
        with serial.Serial(port, BAUD, timeout=0.2) as ser:
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
                    text = line.rstrip(b"\r").decode("utf-8", errors="replace")
                    out.put((name, text))
    except Exception as exc:
        out.put((name, f"MONITOR_ERROR: {exc}"))


def monitor_pair(tx_port: TeensyPort, rx_port: TeensyPort, *, duration_s: float, min_matches: int) -> bool:
    log(f"monitor opening TX={tx_port.device} RX={rx_port.device} baud={BAUD} duration={duration_s:.1f}s")
    events: queue.Queue[tuple[str, str]] = queue.Queue()
    threads = [
        threading.Thread(target=serial_reader, args=("TX", tx_port.device, events), daemon=True),
        threading.Thread(target=serial_reader, args=("RX", rx_port.device, events), daemon=True),
    ]
    for thread in threads:
        thread.start()

    tx_payloads: list[str] = []
    rx_payloads: list[str] = []
    device_failures: list[str] = []
    last_rssi = ""
    last_snr = ""
    tx_re = re.compile(r"^tx=(?P<payload>.+)$")
    rx_re = re.compile(r"^rx_len=(?P<len>\d+)\s+rssi=(?P<rssi>-?\d+)\s+snr=(?P<snr>-?\d+(?:\.\d+)?)\s+data=(?P<payload>.*)$")

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            name, line = events.get(timeout=0.25)
        except queue.Empty:
            continue
        print(f"[{now()}][{name} {tx_port.device if name == 'TX' else rx_port.device}] {line}", flush=True)
        if "FAIL:" in line or "MONITOR_ERROR:" in line:
            device_failures.append(f"{name}: {line}")
        tx_match = tx_re.match(line)
        if name == "TX" and tx_match:
            tx_payloads.append(tx_match.group("payload"))
            continue
        rx_match = rx_re.match(line)
        if name == "RX" and rx_match:
            payload = rx_match.group("payload")
            rx_payloads.append(payload)
            last_rssi = rx_match.group("rssi")
            last_snr = rx_match.group("snr")

    tx_set = set(tx_payloads)
    matched_payloads = [payload for payload in rx_payloads if payload in tx_set]
    missing_payloads = [payload for payload in tx_payloads if payload not in set(rx_payloads)]
    unexpected_payloads = [payload for payload in rx_payloads if payload not in tx_set]
    log("monitor summary")
    log(f"  tx_count={len(tx_payloads)} rx_count={len(rx_payloads)} exact_matches={len(matched_payloads)} required={min_matches}")
    if last_rssi or last_snr:
        log(f"  last_rx_rssi={last_rssi} last_rx_snr={last_snr}")
    if tx_payloads:
        log(f"  last_tx={tx_payloads[-1]}")
    if rx_payloads:
        log(f"  last_rx={rx_payloads[-1]}")
    if matched_payloads:
        log(f"  matched_payloads={', '.join(matched_payloads[-5:])}")
    if missing_payloads:
        log(f"  missing_payloads={', '.join(missing_payloads[:5])}")
    if unexpected_payloads:
        log(f"  unexpected_rx_payloads={', '.join(unexpected_payloads[:5])}")
    if device_failures:
        log(f"  device_failures={len(device_failures)}")
        for failure in device_failures[:5]:
            log(f"    {failure}")

    passed = len(matched_payloads) >= min_matches and not device_failures
    log("RESULT: PASS - TX payloads were received exactly" if passed else "RESULT: FAIL - exact payload match count was too low")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description="Build, upload, and monitor a two-Teensy SX1278 LoRa test.")
    parser.add_argument("--tx", help="Force TX board by /dev/ttyACM* or teensy_ports /sys/... address.")
    parser.add_argument("--rx", help="Force RX board by /dev/ttyACM* or teensy_ports /sys/... address.")
    parser.add_argument("--duration", type=float, default=30.0, help="Serial monitor duration in seconds.")
    parser.add_argument("--min-matches", type=int, default=3, help="Minimum exact TX/RX payload matches required for PASS.")
    parser.add_argument("--build-only", action="store_true", help="Only build both sketches.")
    parser.add_argument("--no-upload", action="store_true", help="Build and monitor existing firmware without uploading.")
    parser.add_argument("--monitor-only", action="store_true", help="Skip build/upload and only monitor two serial Teensys.")
    args = parser.parse_args()

    os.chdir(ROOT)
    log(f"repo={ROOT}")
    log(f"sender_sketch={SENDER_SKETCH}")
    log(f"receiver_sketch={RECEIVER_SKETCH}")
    print_ports("initial USB state")

    sender_hex = BUILD_ROOT / "sender" / ".pio" / "build" / "teensy41" / "firmware.hex"
    receiver_hex = BUILD_ROOT / "receiver" / ".pio" / "build" / "teensy41" / "firmware.hex"

    if not args.monitor_only:
        sender_hex = build_sketch(SENDER_SKETCH, "sender")
        receiver_hex = build_sketch(RECEIVER_SKETCH, "receiver")

    if args.build_only:
        log("build-only requested; stopping before upload/monitor")
        return 0

    if args.monitor_only or args.no_upload:
        serials = wait_for_two_serials(timeout_s=5)
        tx_port, rx_port = serials[0], serials[1]
        if args.tx and args.rx:
            ports = list_teensy_ports()
            tx_port = pick_by_device_or_address(ports, args.tx)
            rx_port = pick_by_device_or_address(ports, args.rx)
        log(f"monitor assignment: TX={tx_port.device} RX={rx_port.device}")
    else:
        tx_port, rx_port = upload_pair(sender_hex, receiver_hex, args)

    print_ports("USB state before monitor")
    ok = monitor_pair(tx_port, rx_port, duration_s=args.duration, min_matches=args.min_matches)
    return 0 if ok else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("interrupted")
        raise SystemExit(130)
    except Exception as exc:
        log(f"ERROR: {exc}")
        raise SystemExit(1)
