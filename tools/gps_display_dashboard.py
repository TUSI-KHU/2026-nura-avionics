#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    platformio_python = Path.home() / ".local" / "share" / "pipx" / "venvs" / "platformio" / "bin" / "python"
    if platformio_python.exists() and Path(sys.executable).resolve() != platformio_python.resolve():
        os.execv(str(platformio_python), [str(platformio_python), *sys.argv])
    raise SystemExit("pyserial is missing. Run this with PlatformIO's Python.") from exc


ROOT = Path(__file__).resolve().parents[1]
TEENSY_VID_PID = {(0x16C0, 0x0483)}
BOOTLOADER_VID_PID = {(0x16C0, 0x0478)}
SERIAL_BAUD = 115200


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NURA Sensor Display</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101417;
      color: #eef3f0;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #101417; }
    main { width: min(1120px, calc(100vw - 32px)); margin: 0 auto; padding: 28px 0 36px; }
    header { display: flex; justify-content: space-between; gap: 16px; align-items: flex-end; margin-bottom: 20px; }
    h1 { font-size: 28px; line-height: 1.1; margin: 0; font-weight: 760; letter-spacing: 0; }
    .sub { margin-top: 8px; color: #9fb2aa; font-size: 14px; }
    .status { display: inline-flex; align-items: center; gap: 9px; min-height: 36px; padding: 0 12px; border: 1px solid #2a3934; border-radius: 6px; background: #151d1a; font-size: 14px; white-space: nowrap; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: #697872; }
    .dot.ok { background: #44d17d; box-shadow: 0 0 18px #44d17d66; }
    .dot.warn { background: #f4b942; box-shadow: 0 0 18px #f4b94266; }
    .grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
    .tile { border: 1px solid #263630; border-radius: 8px; padding: 16px; background: #151b19; min-height: 118px; }
    .wide { grid-column: span 2; }
    .label { color: #91a59c; font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }
    .value { margin-top: 12px; font-size: 30px; line-height: 1; font-weight: 740; overflow-wrap: anywhere; }
    .unit { color: #9fb2aa; font-size: 14px; margin-left: 4px; font-weight: 520; }
    .small { font-size: 18px; line-height: 1.25; }
    .fix { color: #44d17d; }
    .nofix { color: #f4b942; }
    .log { height: 260px; overflow: auto; border: 1px solid #263630; border-radius: 8px; background: #0b0f0d; padding: 12px; font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; color: #b9c8c1; }
    .map { width: 100%; height: 360px; border: 1px solid #263630; border-radius: 8px; background: #0b0f0d; margin: 12px 0; overflow: hidden; }
    .map iframe { width: 100%; height: 100%; border: 0; display: block; }
    .map.empty { display: grid; place-items: center; color: #91a59c; font-size: 14px; }
    .actions { display: flex; gap: 10px; margin: 12px 0 18px; flex-wrap: wrap; }
    a.button { color: #101417; background: #78e0a3; text-decoration: none; border-radius: 6px; padding: 9px 12px; font-weight: 680; font-size: 14px; }
    a.button.disabled { pointer-events: none; background: #2f4039; color: #7d9188; }
    @media (max-width: 760px) {
      header { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .wide { grid-column: span 2; }
      .value { font-size: 24px; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>NURA Sensor Display</h1>
        <div class="sub" id="port">Waiting for serial data</div>
      </div>
      <div class="status"><span class="dot" id="dot"></span><span id="status">Disconnected</span></div>
    </header>

    <section class="grid">
      <div class="tile"><div class="label">Fix</div><div class="value" id="fix">--</div></div>
      <div class="tile"><div class="label">Satellites</div><div class="value" id="sats">--</div></div>
      <div class="tile"><div class="label">HDOP</div><div class="value" id="hdop">--</div></div>
      <div class="tile"><div class="label">NMEA Chars</div><div class="value" id="chars">--</div></div>
      <div class="tile wide"><div class="label">Latitude</div><div class="value small" id="lat">--</div></div>
      <div class="tile wide"><div class="label">Longitude</div><div class="value small" id="lon">--</div></div>
      <div class="tile"><div class="label">Altitude</div><div class="value" id="alt">--</div></div>
      <div class="tile"><div class="label">Speed</div><div class="value" id="speed">--</div></div>
      <div class="tile"><div class="label">Course</div><div class="value" id="course">--</div></div>
      <div class="tile"><div class="label">Checksums</div><div class="value small" id="checksum">--</div></div>
      <div class="tile wide"><div class="label">Low-G Accel</div><div class="value small" id="lowAccel">--</div></div>
      <div class="tile wide"><div class="label">Low-G Gyro</div><div class="value small" id="lowGyro">--</div></div>
      <div class="tile wide"><div class="label">High-G Accel</div><div class="value small" id="highAccel">--</div></div>
      <div class="tile wide"><div class="label">Magnetometer</div><div class="value small" id="mag">--</div></div>
      <div class="tile"><div class="label">Pressure</div><div class="value small" id="pressure">--</div></div>
      <div class="tile"><div class="label">Baro Temp</div><div class="value small" id="baroTemp">--</div></div>
      <div class="tile"><div class="label">Battery</div><div class="value small" id="battery">--</div></div>
      <div class="tile"><div class="label">Sensor Init</div><div class="value small" id="init">--</div></div>
    </section>

    <div class="actions">
      <a class="button disabled" id="maps" href="#" target="_blank" rel="noreferrer">Open Map</a>
    </div>
    <div class="map empty" id="mapBox">Waiting for GPS fix</div>
    <div class="log" id="log"></div>
  </main>
  <script>
    const fields = {
      port: document.getElementById("port"),
      status: document.getElementById("status"),
      dot: document.getElementById("dot"),
      fix: document.getElementById("fix"),
      sats: document.getElementById("sats"),
      hdop: document.getElementById("hdop"),
      chars: document.getElementById("chars"),
      lat: document.getElementById("lat"),
      lon: document.getElementById("lon"),
      alt: document.getElementById("alt"),
      speed: document.getElementById("speed"),
      course: document.getElementById("course"),
      checksum: document.getElementById("checksum"),
      lowAccel: document.getElementById("lowAccel"),
      lowGyro: document.getElementById("lowGyro"),
      highAccel: document.getElementById("highAccel"),
      mag: document.getElementById("mag"),
      pressure: document.getElementById("pressure"),
      baroTemp: document.getElementById("baroTemp"),
      battery: document.getElementById("battery"),
      init: document.getElementById("init"),
      maps: document.getElementById("maps"),
      mapBox: document.getElementById("mapBox"),
      log: document.getElementById("log")
    };

    function fmt(value, digits, empty = "--") {
      return value === null || value === undefined ? empty : Number(value).toFixed(digits);
    }

    function setStatus(text, cls) {
      fields.status.textContent = text;
      fields.dot.className = "dot " + cls;
    }

    function appendLog(line) {
      const div = document.createElement("div");
      div.textContent = line;
      fields.log.appendChild(div);
      while (fields.log.children.length > 120) fields.log.removeChild(fields.log.firstChild);
      fields.log.scrollTop = fields.log.scrollHeight;
    }

    function updateGps(gps) {
      fields.fix.textContent = gps.has_fix ? "FIX" : "NO FIX";
      fields.fix.className = "value " + (gps.has_fix ? "fix" : "nofix");
      fields.sats.textContent = gps.sats ?? "--";
      fields.hdop.textContent = fmt(gps.hdop, 2);
      fields.chars.textContent = gps.chars ?? "--";
      fields.lat.textContent = fmt(gps.lat_deg, 6);
      fields.lon.textContent = fmt(gps.lon_deg, 6);
      fields.alt.innerHTML = `${fmt(gps.alt_m, 1)}<span class="unit">m</span>`;
      fields.speed.innerHTML = `${fmt(gps.speed_mps, 2)}<span class="unit">m/s</span>`;
      fields.course.innerHTML = `${fmt(gps.course_deg, 1)}<span class="unit">deg</span>`;
      fields.checksum.textContent = `${gps.pass_checksum ?? 0} pass / ${gps.fail_checksum ?? 0} fail`;

      if (gps.lat_deg !== null && gps.lon_deg !== null) {
        fields.maps.href = `https://maps.google.com/?q=${gps.lat_deg},${gps.lon_deg}`;
        fields.maps.classList.remove("disabled");
        const lat = Number(gps.lat_deg);
        const lon = Number(gps.lon_deg);
        const delta = 0.004;
        const bbox = [lon - delta, lat - delta, lon + delta, lat + delta].join(",");
        fields.mapBox.className = "map";
        fields.mapBox.innerHTML = `<iframe title="GPS map" src="https://www.openstreetmap.org/export/embed.html?bbox=${bbox}&layer=mapnik&marker=${lat},${lon}"></iframe>`;
      }
    }

    function updateSensorDisplay(data) {
      updateGps(data.gps || {});
      const low = data.low_imu || {};
      const high = data.high_g || {};
      const mag = data.mag || {};
      const baro = data.baro || {};
      const battery = data.battery || {};

      fields.lowAccel.textContent = `${fmt(low.ax_mps2, 2)}, ${fmt(low.ay_mps2, 2)}, ${fmt(low.az_mps2, 2)} m/s2`;
      fields.lowGyro.textContent = `${fmt(low.gx_dps, 2)}, ${fmt(low.gy_dps, 2)}, ${fmt(low.gz_dps, 2)} dps`;
      fields.highAccel.textContent = `${fmt(high.x_g, 2)}, ${fmt(high.y_g, 2)}, ${fmt(high.z_g, 2)} g`;
      fields.mag.textContent = `${fmt(mag.x_ut, 1)}, ${fmt(mag.y_ut, 1)}, ${fmt(mag.z_ut, 1)} uT`;
      fields.pressure.textContent = `${fmt(baro.pressure_pa, 0)} Pa`;
      fields.baroTemp.textContent = `${fmt(baro.temp_c, 1)} C`;
      fields.battery.textContent = battery.valid ? `${battery.battery_mv} mV` : `${battery.battery_mv ?? "--"} mV`;
      const okCount = [low.ok, high.ok, mag.ok, baro.ok, data.gps?.ok, battery.ok].filter(Boolean).length;
      fields.init.textContent = `${okCount}/6 OK`;
      setStatus(data.gps?.has_fix ? "GPS fix" : "Reading sensors", data.gps?.has_fix ? "ok" : "warn");
    }

    function updateTelemetryFast(data) {
      const accel = data.accel_g || [];
      const gyro = data.gyro_dps || [];
      fields.lowAccel.textContent = `${fmt(accel[0], 2)}, ${fmt(accel[1], 2)}, ${fmt(accel[2], 2)} g`;
      fields.lowGyro.textContent = `${fmt(gyro[0], 2)}, ${fmt(gyro[1], 2)}, ${fmt(gyro[2], 2)} dps`;
      fields.pressure.textContent = `${data.baro_dp_2pa * 2} Pa delta`;
      fields.battery.textContent = `${data.battery_mv ?? "--"} mV`;
      fields.init.textContent = data.health || "--";
      const radio = data.radio_ok ? "radio ok" : "radio missing";
      setStatus(`${data.state || "telemetry"} / ${radio} / RSSI ${data.rssi ?? "--"} dBm`, data.baro_ok && data.imu_ok && data.radio_ok ? "ok" : "warn");
    }

    function updateTelemetryStatus(data) {
      const fail = data.decode_fail ?? 0;
      setStatus(`radio ${data.radio || "--"} / rx ${data.phy_rx ?? "--"} / decode_fail ${fail}`, fail === 0 ? "ok" : "warn");
    }

    const events = new EventSource("/events");
    events.addEventListener("message", (event) => {
      const msg = JSON.parse(event.data);
      if (msg.type === "meta") {
        fields.port.textContent = msg.text;
        setStatus(msg.status, msg.ok ? "ok" : "warn");
        appendLog(msg.text);
        return;
      }
      if (msg.type === "init") {
        const d = msg.data;
        const okCount = [d.low_imu, d.high_g, d.mag, d.baro, d.gps, d.battery].filter(Boolean).length;
        fields.init.textContent = `${okCount}/6 OK`;
        appendLog(msg.line || JSON.stringify(msg));
        return;
      }
      if (msg.type === "sensors") {
        updateSensorDisplay(msg.data);
        appendLog(msg.line);
        return;
      }
      if (msg.type === "telemetry_fast") {
        updateTelemetryFast(msg.data);
        appendLog(msg.line);
        return;
      }
      if (msg.type === "telemetry_status") {
        updateTelemetryStatus(msg.data);
        appendLog(msg.line);
        return;
      }
      if (msg.type !== "gps") {
        appendLog(msg.line || JSON.stringify(msg));
        return;
      }

      const gps = msg.data;
      updateGps(gps);
      setStatus(gps.has_fix ? "GPS fix" : "Reading GPS", gps.has_fix ? "ok" : "warn");
      appendLog(msg.line);
    });

    events.onerror = () => setStatus("Dashboard waiting", "warn");
  </script>
</body>
</html>
"""


class Hub:
    def __init__(self) -> None:
        self.clients: list[queue.Queue[dict[str, object]]] = []
        self.lock = threading.Lock()

    def publish(self, message: dict[str, object]) -> None:
        with self.lock:
            clients = list(self.clients)
        for client in clients:
            try:
                client.put_nowait(message)
            except queue.Full:
                pass

    def subscribe(self) -> queue.Queue[dict[str, object]]:
        client: queue.Queue[dict[str, object]] = queue.Queue(maxsize=64)
        with self.lock:
            self.clients.append(client)
        return client

    def unsubscribe(self, client: queue.Queue[dict[str, object]]) -> None:
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)


def log(message: str) -> None:
    print(f"[{dt.datetime.now().strftime('%H:%M:%S')}] {message}", flush=True)


def teensy_serial_ports() -> list[str]:
    ports: list[str] = []
    for port in list_ports.comports():
        vid_pid = (port.vid, port.pid)
        if vid_pid in TEENSY_VID_PID or "Teensy" in (port.description or ""):
            if port.device.startswith("/dev/tty"):
                ports.append(port.device)
    return sorted(set(ports))


def bootloader_present() -> bool:
    return any((port.vid, port.pid) in BOOTLOADER_VID_PID for port in list_ports.comports())


def wait_for_serial_port(timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ports = teensy_serial_ports()
        if ports:
            return ports[0]
        time.sleep(0.25)
    return None


def upload_firmware() -> None:
    cmd = ["pio", "run", "-e", "sensor_display_test", "-t", "upload"]
    log("$ " + " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


FAST_RE = re.compile(
    r"^rx type=FAST\b.*?\bseq=(?P<seq>\d+).*?\bstate=(?P<state>\S+).*?"
    r"\bbaro_dp_2pa=(?P<baro>-?\d+).*?"
    r"\baccel_g=\((?P<ax>-?[\d.]+),(?P<ay>-?[\d.]+),(?P<az>-?[\d.]+)\).*?"
    r"\bgyro_dps=\((?P<gx>-?[\d.]+),(?P<gy>-?[\d.]+),(?P<gz>-?[\d.]+)\).*?"
    r"\bbatt_mv=(?P<batt>\d+).*?\bhealth=(?P<health>\S+).*?"
    r"\brssi=(?P<rssi>-?\d+).*?\bsnr=(?P<snr>-?[\d.]+)"
)

GPS_RE = re.compile(
    r"^rx type=GPS\b.*?\bseq=(?P<seq>\d+).*?\bfix=(?P<fix>\S+).*?"
    r"\blat_deg=(?P<lat>-?[\d.]+).*?\blon_deg=(?P<lon>-?[\d.]+).*?"
    r"\balt_m=(?P<alt>-?[\d.]+).*?\bspeed_mps=(?P<speed>-?[\d.]+).*?"
    r"\bcourse_deg=(?P<course>-?[\d.]+).*?\bhdop=(?P<hdop>-?[\d.]+).*?"
    r"\bsats=(?P<sats>\d+).*?\brssi=(?P<rssi>-?\d+).*?\bsnr=(?P<snr>-?[\d.]+)"
)

STATUS_RE = re.compile(
    r"^status\b.*?\bradio=(?P<radio>\S+).*?\bphy_rx=(?P<phy_rx>\d+).*?"
    r"\bdecode_fail=(?P<decode_fail>\d+).*?\brssi_now_dbm=(?P<rssi>-?\d+)"
)


def parse_receiver_line(line: str) -> dict[str, object] | None:
    match = FAST_RE.search(line)
    if match:
        data = match.groupdict()
        health = data["health"].split(",")
        return {
            "type": "telemetry_fast",
            "line": line,
            "data": {
                "seq": int(data["seq"]),
                "state": data["state"],
                "baro_dp_2pa": int(data["baro"]),
                "accel_g": [float(data["ax"]), float(data["ay"]), float(data["az"])],
                "gyro_dps": [float(data["gx"]), float(data["gy"]), float(data["gz"])],
                "battery_mv": int(data["batt"]),
                "health": data["health"],
                "imu_ok": "imu" in health,
                "baro_ok": "baro" in health,
                "radio_ok": "radio" in health,
                "rssi": int(data["rssi"]),
                "snr": float(data["snr"]),
            },
        }

    match = GPS_RE.search(line)
    if match:
        data = match.groupdict()
        has_fix = data["fix"].lower() == "yes"
        return {
            "type": "gps",
            "line": line,
            "data": {
                "has_fix": has_fix,
                "lat_deg": float(data["lat"]) if has_fix else None,
                "lon_deg": float(data["lon"]) if has_fix else None,
                "alt_m": float(data["alt"]),
                "speed_mps": float(data["speed"]),
                "course_deg": float(data["course"]),
                "hdop": float(data["hdop"]),
                "sats": int(data["sats"]),
                "chars": None,
                "pass_checksum": int(data["seq"]),
                "fail_checksum": 0,
            },
        }

    match = STATUS_RE.search(line)
    if match:
        data = match.groupdict()
        return {
            "type": "telemetry_status",
            "line": line,
            "data": {
                "radio": data["radio"],
                "phy_rx": int(data["phy_rx"]),
                "decode_fail": int(data["decode_fail"]),
                "rssi": int(data["rssi"]),
            },
        }

    return None


def serial_worker(hub: Hub, port: str | None, baud: int, stop: threading.Event) -> None:
    selected_port = port or wait_for_serial_port(2.0)
    if selected_port is None:
        hint = "No Teensy serial port found"
        if bootloader_present():
            hint += "; bootloader is visible, upload sensor_display_test first"
        hub.publish({"type": "meta", "ok": False, "status": "No serial", "text": hint})
        return

    hub.publish({"type": "meta", "ok": True, "status": "Serial connected", "text": f"OPEN {selected_port} baud={baud}"})
    log(f"opening {selected_port} at {baud}")

    with serial.Serial(selected_port, baud, timeout=0.2) as ser:
        ser.dtr = True
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            receiver_message = parse_receiver_line(line)
            if receiver_message is not None:
                hub.publish(receiver_message)
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                hub.publish({"type": "text", "line": line})
                continue
            src = data.get("src")
            if src == "gnss_state":
                hub.publish({"type": "gps", "line": line, "data": data})
            elif src == "sensor_display_init":
                hub.publish({"type": "init", "line": line, "data": data})
            elif src == "sensor_display":
                hub.publish({"type": "sensors", "line": line, "data": data})
            else:
                hub.publish({"type": "json", "line": line, "data": data})


def make_handler(hub: Hub) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/":
                body = HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/events":
                client = hub.subscribe()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while True:
                        message = client.get(timeout=15.0)
                        payload = json.dumps(message, separators=(",", ":"))
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                except queue.Empty:
                    pass
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    hub.unsubscribe(client)
                return

            self.send_error(HTTPStatus.NOT_FOUND)

        def log_message(self, format: str, *args: object) -> None:
            return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Web display for the NURA sensor JSON serial test.")
    parser.add_argument("--port", help="Serial port, for example /dev/ttyACM0.")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--upload", action="store_true", help="Build and upload the sensor_display_test firmware first.")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    if args.upload:
        upload_firmware()
        args.port = args.port or wait_for_serial_port(12.0)

    hub = Hub()
    stop = threading.Event()
    worker = threading.Thread(target=serial_worker, args=(hub, args.port, args.baud, stop), daemon=True)
    worker.start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(hub))
    url = f"http://{args.host}:{args.http_port}/"
    log(f"dashboard: {url}")
    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("stopping")
    finally:
        stop.set()
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
