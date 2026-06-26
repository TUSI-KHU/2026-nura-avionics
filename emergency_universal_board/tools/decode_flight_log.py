#!/usr/bin/env python3
"""Decode NURA binary flight logs from SD files or raw SPI flash dumps."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path


FRAME_MAGIC = 0x4E4C
FRAME_VERSION = 1
FRAME_HEADER = struct.Struct("<HBBHII")
CRC16 = struct.Struct("<H")

FLASH_SECTOR_MAGIC = 0x4E4C4653
FLASH_SECTOR_HEADER = struct.Struct("<IHHIIIII")
FLASH_PAYLOAD_OFFSET = 64

TYPE_NAMES = {
    1: "FAST",
    2: "SLOW",
    3: "EVENT",
    4: "DECISION",
}

STATE_NAMES = {
    0: "INIT",
    1: "SAFE",
    2: "ARMED",
    3: "LAUNCH",
    4: "COAST",
    5: "APOGEE",
    6: "DROGUE",
    7: "DEPLOY",
    8: "GROUND",
    9: "FAULT",
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def maybe_flash_sector_header(data: bytes, offset: int, sector_bytes: int) -> bool:
    if offset + FLASH_SECTOR_HEADER.size > len(data):
        return False
    fields = FLASH_SECTOR_HEADER.unpack_from(data, offset)
    magic, version, header_bytes, stored_sector_bytes, payload_offset, session_id, _, header_crc = fields
    return (
        magic == FLASH_SECTOR_MAGIC
        and version == 1
        and header_bytes == FLASH_SECTOR_HEADER.size
        and stored_sector_bytes == sector_bytes
        and payload_offset == FLASH_PAYLOAD_OFFSET
        and session_id not in (0, 0xFFFFFFFF)
        and header_crc not in (0, 0xFFFFFFFF)
    )


def summarize_payload(record_type: int, payload: bytes) -> str:
    try:
        if record_type == 1 and len(payload) >= 66:
            state = payload[0]
            pressure_pa = struct.unpack_from("<i", payload, 52)[0]
            filtered_cm = struct.unpack_from("<i", payload, 60)[0]
            batt_mv = struct.unpack_from("<H", payload, 64)[0]
            return f"state={STATE_NAMES.get(state, state)} pressure_pa={pressure_pa} alt_m={filtered_cm / 100:.2f} batt_mv={batt_mv}"
        if record_type == 2 and len(payload) >= 58:
            state = payload[0]
            lat = struct.unpack_from("<i", payload, 24)[0] / 1e7
            lon = struct.unpack_from("<i", payload, 28)[0] / 1e7
            sats = payload[42]
            return f"state={STATE_NAMES.get(state, state)} lat={lat:.7f} lon={lon:.7f} sats={sats}"
        if record_type == 3 and len(payload) >= 12:
            event_id, prev_state, curr_state, _ = struct.unpack_from("<BBBB", payload, 0)
            data0, data1 = struct.unpack_from("<II", payload, 4)
            return f"event={event_id} prev={STATE_NAMES.get(prev_state, prev_state)} curr={STATE_NAMES.get(curr_state, curr_state)} data0={data0} data1={data1}"
        if record_type == 4 and len(payload) >= 30:
            seq = struct.unpack_from("<I", payload, 0)[0]
            state, kind, result, count0, count1, _ = struct.unpack_from("<BBBBBB", payload, 4)
            reason = struct.unpack_from("<H", payload, 10)[0]
            values = struct.unpack_from("<ffff", payload, 12)
            return (
                f"decision={seq} state={STATE_NAMES.get(state, state)} kind={kind} "
                f"result={result} reason=0x{reason:04x} count0={count0} count1={count1} "
                f"v0={values[0]:.3f} v1={values[1]:.3f} v2={values[2]:.3f} v3={values[3]:.3f}"
            )
    except struct.error:
        pass
    return f"payload_len={len(payload)}"


def decode(data: bytes, sector_bytes: int):
    offset = 0
    while offset + FRAME_HEADER.size + CRC16.size <= len(data):
        if maybe_flash_sector_header(data, offset, sector_bytes):
            sector_end = ((offset // sector_bytes) + 1) * sector_bytes
            offset = min(offset + FLASH_PAYLOAD_OFFSET, sector_end)
            continue

        magic, version, record_type, payload_len, sequence, timestamp_ms = FRAME_HEADER.unpack_from(data, offset)
        if magic != FRAME_MAGIC or version != FRAME_VERSION:
            if data[offset] == 0xFF:
                next_data = data.find(b"\x4c\x4e", offset + 1)
                if next_data == -1:
                    break
                offset = next_data
            else:
                offset += 1
            continue

        frame_len = FRAME_HEADER.size + payload_len + CRC16.size
        if offset + frame_len > len(data):
            break

        frame = data[offset : offset + frame_len]
        expected_crc = CRC16.unpack_from(frame, FRAME_HEADER.size + payload_len)[0]
        actual_crc = crc16_ccitt(frame[: FRAME_HEADER.size + payload_len])
        if expected_crc != actual_crc:
            offset += 1
            continue

        payload = frame[FRAME_HEADER.size : FRAME_HEADER.size + payload_len]
        yield {
            "offset": offset,
            "sequence": sequence,
            "timestamp_ms": timestamp_ms,
            "type": TYPE_NAMES.get(record_type, str(record_type)),
            "payload_len": payload_len,
            "summary": summarize_payload(record_type, payload),
        }
        offset += frame_len


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--sector-bytes", type=int, default=4096)
    parser.add_argument("--csv", type=Path, help="Write decoded row summaries to CSV")
    args = parser.parse_args()

    data = args.input.read_bytes()
    rows = list(decode(data, args.sector_bytes))

    if args.csv:
        with args.csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["offset", "sequence", "timestamp_ms", "type", "payload_len", "summary"])
            writer.writeheader()
            writer.writerows(rows)
    else:
        writer = csv.DictWriter(sys.stdout, fieldnames=["offset", "sequence", "timestamp_ms", "type", "payload_len", "summary"])
        writer.writeheader()
        writer.writerows(rows)

    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
