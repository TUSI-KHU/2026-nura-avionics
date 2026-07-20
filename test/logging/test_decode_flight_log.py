#!/usr/bin/env python3
from __future__ import annotations

import struct
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import decode_flight_log as decoder  # noqa: E402


def with_crc(header: bytearray, offset: int) -> bytes:
    header[offset : offset + 2] = b"\x00\x00"
    struct.pack_into("<H", header, offset, decoder.crc16_ccitt(header))
    return bytes(header)


def make_frame(sequence: int) -> bytes:
    payload = struct.pack("<BBBBII", 2, 1, 2, 0, 123, 456)
    header = decoder.FRAME_HEADER.pack(
        decoder.FRAME_MAGIC,
        decoder.FRAME_VERSION,
        3,
        len(payload),
        sequence,
        1000 + sequence,
    )
    return header + payload + decoder.CRC16.pack(decoder.crc16_ccitt(header + payload))


def main() -> int:
    sector = bytearray(b"\xff" * 4096)
    sector_header = bytearray(
        decoder.FLASH_SECTOR_HEADER.pack(
            decoder.FLASH_SECTOR_MAGIC,
            7,
            0,
            decoder.FLASH_JOURNAL_VERSION,
            0,
        )
    )
    sector[: len(sector_header)] = with_crc(sector_header, 14)

    stream = make_frame(1) + make_frame(2)
    page_header = bytearray(
        decoder.FLASH_PAGE_HEADER.pack(
            decoder.FLASH_PAGE_MAGIC,
            0,
            len(stream),
            decoder.crc16_ccitt(stream),
            0,
            decoder.FLASH_JOURNAL_VERSION,
            0,
        )
    )
    page_header = bytearray(with_crc(page_header, 12))
    sector[256 : 256 + len(page_header)] = page_header
    sector[256 + len(page_header) : 256 + len(page_header) + len(stream)] = stream

    extracted = decoder.extract_flash_journal(bytes(sector), 4096)
    assert extracted == stream
    rows = list(decoder.decode(extracted, 4096))
    assert [row["sequence"] for row in rows] == [1, 2]

    sector[256 + decoder.FLASH_PAGE_HEADER.size] ^= 0x01
    assert decoder.extract_flash_journal(bytes(sector), 4096) == b""

    sd_block = bytearray(b"\xff" * decoder.SD_BLOCK_BYTES)
    session_id = 0x12345678
    sd_header = bytearray(
        decoder.SD_BLOCK_HEADER.pack(
            decoder.SD_BLOCK_MAGIC,
            session_id,
            0,
            0,
            len(stream),
            decoder.crc16_ccitt(stream),
            0,
            decoder.SD_BLOCK_VERSION,
            0,
        )
    )
    sd_header = bytearray(with_crc(sd_header, 20))
    sd_block[: len(sd_header)] = sd_header
    sd_block[len(sd_header) : len(sd_header) + len(stream)] = stream
    stale_tail = b"stale-data" * 200
    extracted_sd = decoder.extract_sd_blocks(bytes(sd_block) + stale_tail)
    assert extracted_sd == stream
    assert [row["sequence"] for row in decoder.decode(extracted_sd, 4096)] == [1, 2]

    print("flight log decoder tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
