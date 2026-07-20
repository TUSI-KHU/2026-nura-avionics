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

FLASH_SECTOR_MAGIC = 0x4E534543
FLASH_PAGE_MAGIC = 0x4E504147
FLASH_JOURNAL_VERSION = 1
FLASH_SECTOR_HEADER = struct.Struct("<IIIHH")
FLASH_PAGE_HEADER = struct.Struct("<IIHHHBB")
FLASH_PAGE_BYTES = 256
SD_BLOCK_MAGIC = 0x4E534442
SD_BLOCK_VERSION = 1
SD_BLOCK_BYTES = 512
SD_BLOCK_HEADER = struct.Struct("<IIIIHHHBB")

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


def valid_flash_sector_header(data: bytes, offset: int) -> tuple[int, int] | None:
    if offset + FLASH_SECTOR_HEADER.size > len(data):
        return None
    magic, sequence, stream_offset, version, stored_crc = FLASH_SECTOR_HEADER.unpack_from(data, offset)
    if magic != FLASH_SECTOR_MAGIC or version != FLASH_JOURNAL_VERSION:
        return None
    header = bytearray(data[offset : offset + FLASH_SECTOR_HEADER.size])
    header[14:16] = b"\x00\x00"
    if crc16_ccitt(header) != stored_crc:
        return None
    return sequence, stream_offset


def valid_flash_page(page: bytes) -> tuple[int, bytes] | None:
    if len(page) != FLASH_PAGE_BYTES:
        return None
    magic, stream_offset, payload_len, payload_crc, header_crc, version, _ = FLASH_PAGE_HEADER.unpack_from(page)
    if (
        magic != FLASH_PAGE_MAGIC
        or version != FLASH_JOURNAL_VERSION
        or payload_len == 0
        or payload_len > FLASH_PAGE_BYTES - FLASH_PAGE_HEADER.size
    ):
        return None
    header = bytearray(page[: FLASH_PAGE_HEADER.size])
    header[12:14] = b"\x00\x00"
    if crc16_ccitt(header) != header_crc:
        return None
    payload = page[FLASH_PAGE_HEADER.size : FLASH_PAGE_HEADER.size + payload_len]
    if crc16_ccitt(payload) != payload_crc:
        return None
    return stream_offset, payload


def extract_flash_journal(data: bytes, sector_bytes: int) -> bytes | None:
    sectors: list[tuple[int, int]] = []
    for base in range(0, len(data) - FLASH_SECTOR_HEADER.size + 1, sector_bytes):
        header = valid_flash_sector_header(data, base)
        if header is not None:
            sequence, _ = header
            sectors.append((sequence, base))
    if not sectors:
        return None

    pages: list[tuple[int, bytes]] = []
    for _, base in sorted(sectors):
        for page_offset in range(FLASH_PAGE_BYTES, sector_bytes, FLASH_PAGE_BYTES):
            start = base + page_offset
            end = start + FLASH_PAGE_BYTES
            if end > len(data):
                break
            page = data[start:end]
            if page == b"\xff" * FLASH_PAGE_BYTES:
                break
            valid = valid_flash_page(page)
            if valid is not None:
                pages.append(valid)

    if not pages:
        return b""

    stream = bytearray()
    expected_offset: int | None = None
    for stream_offset, payload in sorted(pages):
        if expected_offset is not None and stream_offset > expected_offset:
            stream.extend(b"\xff" * 16)
        if expected_offset is not None and stream_offset < expected_offset:
            overlap = expected_offset - stream_offset
            if overlap >= len(payload):
                continue
            payload = payload[overlap:]
        stream.extend(payload)
        expected_offset = stream_offset + len(payload)
    return bytes(stream)


def valid_sd_block(block: bytes) -> tuple[int, int, int, bytes] | None:
    if len(block) != SD_BLOCK_BYTES:
        return None
    fields = SD_BLOCK_HEADER.unpack_from(block)
    magic, session_id, block_sequence, stream_offset, payload_len, payload_crc, header_crc, version, _ = fields
    if (
        magic != SD_BLOCK_MAGIC
        or version != SD_BLOCK_VERSION
        or session_id in (0, 0xFFFFFFFF)
        or payload_len == 0
        or payload_len > SD_BLOCK_BYTES - SD_BLOCK_HEADER.size
    ):
        return None
    header = bytearray(block[: SD_BLOCK_HEADER.size])
    header[20:22] = b"\x00\x00"
    if crc16_ccitt(header) != header_crc:
        return None
    payload = block[SD_BLOCK_HEADER.size : SD_BLOCK_HEADER.size + payload_len]
    if crc16_ccitt(payload) != payload_crc:
        return None
    return session_id, block_sequence, stream_offset, payload


def extract_sd_blocks(data: bytes) -> bytes | None:
    if len(data) < SD_BLOCK_BYTES:
        return None
    first = valid_sd_block(data[:SD_BLOCK_BYTES])
    if first is None or first[1] != 0 or first[2] != 0:
        return None

    session_id = first[0]
    stream = bytearray()
    expected_sequence = 0
    expected_offset = 0
    for base in range(0, len(data) - SD_BLOCK_BYTES + 1, SD_BLOCK_BYTES):
        valid = valid_sd_block(data[base : base + SD_BLOCK_BYTES])
        if valid is None:
            break
        block_session, block_sequence, stream_offset, payload = valid
        if (
            block_session != session_id
            or block_sequence != expected_sequence
            or stream_offset != expected_offset
        ):
            break
        stream.extend(payload)
        expected_sequence += 1
        expected_offset += len(payload)
    return bytes(stream)


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
    sd_stream = extract_sd_blocks(data)
    if sd_stream is not None:
        data = sd_stream
    else:
        journal_stream = extract_flash_journal(data, args.sector_bytes)
        if journal_stream is not None:
            data = journal_stream
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
