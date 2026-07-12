#!/usr/bin/env python3
"""Convert NURA .NLG flight log frames into readable CSV and JSON files."""

from __future__ import annotations

import argparse
import csv
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any


FRAME_MAGIC = 0x4E4C
FRAME_VERSION = 1
MAX_FRAME_BYTES = 256

HEADER_STRUCT = struct.Struct("<HBBHII")
FAST_STRUCT = struct.Struct("<BBHII3h3h4hI3h3hIiiiH")
SLOW_STRUCT = struct.Struct("<BBHI3h3hIiiihhHBIIIHBBB")
EVENT_STRUCT = struct.Struct("<BBBBII")
DECISION_STRUCT = struct.Struct("<IBBBBBBHffff")

RECORD_TYPES = {
    1: "FAST_SAMPLE",
    2: "SLOW_SAMPLE",
    3: "EVENT",
    4: "DECISION",
}

STATES = {
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

EVENT_IDS = {
    1: "BOOT",
    2: "STATE_TRANSITION",
    3: "GROUND_STOP",
    4: "STORAGE_FAULT",
}

DECISION_KINDS = {
    0: "NONE",
    1: "LAUNCH_ACCEL",
    2: "BURNOUT_ACCEL",
    3: "APOGEE_PREDICTION",
    4: "APOGEE_DESCENT",
    5: "APOGEE_TIMER",
    6: "BARO_FAULT_TILT",
    7: "FORCE_DEPLOY",
    8: "MAIN_DEPLOY",
    9: "LANDING",
}

DECISION_RESULTS = {
    0: "OBSERVE",
    1: "REJECT",
    2: "ACCEPT",
}

DECISION_REASON_BITS = {
    1 << 0: "PRIMARY_SENSOR",
    1 << 1: "FALLBACK_SENSOR",
    1 << 2: "THRESHOLD_MET",
    1 << 3: "THRESHOLD_NOT_MET",
    1 << 4: "CONFIRMATION_MET",
    1 << 5: "TOO_EARLY",
    1 << 6: "SENSOR_FAULT",
    1 << 7: "TIMEOUT",
    1 << 8: "QUALITY_REJECT",
    1 << 9: "FORCED",
}

FAST_FLAG_BITS = {
    1 << 0: "LOW_IMU_UPDATED",
    1 << 1: "ATTITUDE_VALID",
    1 << 2: "TILT_VALID",
    1 << 3: "BARO_VALID",
    1 << 4: "BARO_REFERENCE_VALID",
    1 << 5: "BARO_FAULT",
    1 << 6: "GPS_FIX",
}

GPS_FLAG_BITS = {
    1 << 0: "GPS_UPDATED",
    1 << 1: "GPS_FIX",
}

HEALTH_FLAG_BITS = {
    1 << 0: "HIGH_ACCEL_OK",
    1 << 1: "MAG_OK",
    1 << 2: "STORAGE_OK",
    1 << 3: "PYRO_CONTINUITY_OK",
    1 << 4: "DEPLOY_FIRED",
}


@dataclass
class ParsedFrame:
    offset: int
    sequence: int
    timestamp_ms: int
    record_type_id: int
    record_type: str
    payload_length: int
    crc: int
    payload: dict[str, Any]


def bit_names(value: int, table: dict[int, str]) -> str:
    names = [name for bit, name in table.items() if value & bit]
    return "|".join(names) if names else "NONE"


def mapped_name(value: int, table: dict[int, str]) -> str:
    return table.get(value, f"UNKNOWN_{value}")


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


def parse_fast(payload: bytes) -> dict[str, Any]:
    (
        state,
        flags,
        health_flags,
        decision_seq,
        low_imu_updated_ms,
        low_ax_mg,
        low_ay_mg,
        low_az_mg,
        gyro_x_dps10,
        gyro_y_dps10,
        gyro_z_dps10,
        roll_deg10,
        pitch_deg10,
        yaw_deg10,
        tilt_deg10,
        high_g_updated_ms,
        high_raw_x,
        high_raw_y,
        high_raw_z,
        high_ax_mg,
        high_ay_mg,
        high_az_mg,
        baro_updated_ms,
        pressure_pa,
        raw_altitude_cm,
        filtered_altitude_cm,
        battery_mv,
    ) = FAST_STRUCT.unpack(payload)

    return {
        "state_id": state,
        "state": mapped_name(state, STATES),
        "flags_raw": flags,
        "flags": bit_names(flags, FAST_FLAG_BITS),
        "health_flags_raw": health_flags,
        "health_flags": bit_names(health_flags, HEALTH_FLAG_BITS),
        "decision_seq": decision_seq,
        "low_imu_updated_ms": low_imu_updated_ms,
        "low_accel_x_g": low_ax_mg / 1000.0,
        "low_accel_y_g": low_ay_mg / 1000.0,
        "low_accel_z_g": low_az_mg / 1000.0,
        "low_gyro_x_dps": gyro_x_dps10 / 10.0,
        "low_gyro_y_dps": gyro_y_dps10 / 10.0,
        "low_gyro_z_dps": gyro_z_dps10 / 10.0,
        "roll_deg": roll_deg10 / 10.0,
        "pitch_deg": pitch_deg10 / 10.0,
        "yaw_deg": yaw_deg10 / 10.0,
        "tilt_deg": tilt_deg10 / 10.0,
        "high_g_updated_ms": high_g_updated_ms,
        "high_raw_x": high_raw_x,
        "high_raw_y": high_raw_y,
        "high_raw_z": high_raw_z,
        "high_accel_x_g": high_ax_mg / 1000.0,
        "high_accel_y_g": high_ay_mg / 1000.0,
        "high_accel_z_g": high_az_mg / 1000.0,
        "baro_updated_ms": baro_updated_ms,
        "pressure_pa": pressure_pa,
        "raw_altitude_m": raw_altitude_cm / 100.0,
        "filtered_altitude_m": filtered_altitude_cm / 100.0,
        "battery_mv": battery_mv,
    }


def parse_slow(payload: bytes) -> dict[str, Any]:
    (
        state,
        gps_flags,
        health_flags,
        mag_updated_ms,
        mag_raw_x,
        mag_raw_y,
        mag_raw_z,
        mag_x_ut10,
        mag_y_ut10,
        mag_z_ut10,
        gps_updated_ms,
        latitude_e7,
        longitude_e7,
        gps_altitude_cm,
        gps_speed_cms,
        gps_course_deg10,
        gps_hdop100,
        gps_satellites,
        gps_chars_processed,
        gps_passed_checksum,
        gps_failed_checksum,
        baro_fault_flags,
        baro_consecutive_read_fail_count,
        baro_consecutive_bad_value_count,
        baro_total_bad_value_count,
    ) = SLOW_STRUCT.unpack(payload)

    return {
        "state_id": state,
        "state": mapped_name(state, STATES),
        "gps_flags_raw": gps_flags,
        "gps_flags": bit_names(gps_flags, GPS_FLAG_BITS),
        "health_flags_raw": health_flags,
        "health_flags": bit_names(health_flags, HEALTH_FLAG_BITS),
        "mag_updated_ms": mag_updated_ms,
        "mag_raw_x": mag_raw_x,
        "mag_raw_y": mag_raw_y,
        "mag_raw_z": mag_raw_z,
        "mag_x_ut": mag_x_ut10 / 10.0,
        "mag_y_ut": mag_y_ut10 / 10.0,
        "mag_z_ut": mag_z_ut10 / 10.0,
        "gps_updated_ms": gps_updated_ms,
        "latitude_deg": latitude_e7 / 10_000_000.0,
        "longitude_deg": longitude_e7 / 10_000_000.0,
        "gps_altitude_m": gps_altitude_cm / 100.0,
        "gps_speed_mps": gps_speed_cms / 100.0,
        "gps_course_deg": gps_course_deg10 / 10.0,
        "gps_hdop": gps_hdop100 / 100.0,
        "gps_satellites": gps_satellites,
        "gps_chars_processed": gps_chars_processed,
        "gps_passed_checksum": gps_passed_checksum,
        "gps_failed_checksum": gps_failed_checksum,
        "baro_fault_flags": baro_fault_flags,
        "baro_consecutive_read_fail_count": baro_consecutive_read_fail_count,
        "baro_consecutive_bad_value_count": baro_consecutive_bad_value_count,
        "baro_total_bad_value_count": baro_total_bad_value_count,
    }


def parse_event(payload: bytes) -> dict[str, Any]:
    event_id, previous_state, current_state, reserved, data0, data1 = EVENT_STRUCT.unpack(payload)
    return {
        "event_id": event_id,
        "event": mapped_name(event_id, EVENT_IDS),
        "previous_state_id": previous_state,
        "previous_state": mapped_name(previous_state, STATES),
        "current_state_id": current_state,
        "current_state": mapped_name(current_state, STATES),
        "reserved": reserved,
        "data0": data0,
        "data1": data1,
    }


def parse_decision(payload: bytes) -> dict[str, Any]:
    (
        decision_seq,
        state,
        kind,
        result,
        count0,
        count1,
        reserved,
        reason,
        value0,
        value1,
        value2,
        value3,
    ) = DECISION_STRUCT.unpack(payload)
    return {
        "decision_seq": decision_seq,
        "state_id": state,
        "state": mapped_name(state, STATES),
        "kind_id": kind,
        "kind": mapped_name(kind, DECISION_KINDS),
        "result_id": result,
        "result": mapped_name(result, DECISION_RESULTS),
        "count0": count0,
        "count1": count1,
        "reserved": reserved,
        "reason_raw": reason,
        "reason": bit_names(reason, DECISION_REASON_BITS),
        "value0": value0,
        "value1": value1,
        "value2": value2,
        "value3": value3,
    }


PAYLOAD_PARSERS = {
    1: (FAST_STRUCT, parse_fast),
    2: (SLOW_STRUCT, parse_slow),
    3: (EVENT_STRUCT, parse_event),
    4: (DECISION_STRUCT, parse_decision),
}


def parse_nlg(data: bytes) -> list[ParsedFrame]:
    frames: list[ParsedFrame] = []
    offset = 0
    while offset < len(data):
        if len(data) - offset < HEADER_STRUCT.size:
            raise ValueError(f"torn frame header at byte offset {offset}")

        magic, version, record_type_id, payload_length, sequence, timestamp_ms = HEADER_STRUCT.unpack_from(data, offset)
        if magic != FRAME_MAGIC:
            raise ValueError(f"bad frame magic 0x{magic:04x} at byte offset {offset}")
        if version != FRAME_VERSION:
            raise ValueError(f"unsupported frame version {version} at byte offset {offset}")

        frame_length = HEADER_STRUCT.size + payload_length + 2
        if frame_length > MAX_FRAME_BYTES:
            raise ValueError(f"frame too long ({frame_length} bytes) at byte offset {offset}")
        if offset + frame_length > len(data):
            raise ValueError(f"torn frame body at byte offset {offset}")

        frame = data[offset : offset + frame_length]
        stored_crc = struct.unpack_from("<H", frame, frame_length - 2)[0]
        calculated_crc = crc16_ccitt(frame[:-2])
        if stored_crc != calculated_crc:
            raise ValueError(
                f"CRC mismatch at byte offset {offset}: stored=0x{stored_crc:04x}, calculated=0x{calculated_crc:04x}"
            )

        payload = frame[HEADER_STRUCT.size : -2]
        record_type = mapped_name(record_type_id, RECORD_TYPES)
        parser_entry = PAYLOAD_PARSERS.get(record_type_id)
        if parser_entry is None:
            parsed_payload = {"payload_hex": payload.hex()}
        else:
            expected_struct, payload_parser = parser_entry
            if payload_length != expected_struct.size:
                raise ValueError(
                    f"{record_type} payload length mismatch at byte offset {offset}: "
                    f"got {payload_length}, expected {expected_struct.size}"
                )
            parsed_payload = payload_parser(payload)

        frames.append(
            ParsedFrame(
                offset=offset,
                sequence=sequence,
                timestamp_ms=timestamp_ms,
                record_type_id=record_type_id,
                record_type=record_type,
                payload_length=payload_length,
                crc=stored_crc,
                payload=parsed_payload,
            )
        )
        offset += frame_length
    return frames


def row_for_frame(frame: ParsedFrame) -> dict[str, Any]:
    row = {
        "offset": frame.offset,
        "sequence": frame.sequence,
        "timestamp_ms": frame.timestamp_ms,
        "record_type_id": frame.record_type_id,
        "record_type": frame.record_type,
        "payload_length": frame.payload_length,
        "crc_hex": f"0x{frame.crc:04X}",
    }
    row.update(frame.payload)
    return row


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return

    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fieldnames.append(key)
                seen.add(key)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def default_output_dir(input_path: Path) -> Path:
    return input_path.with_suffix("").with_name(f"{input_path.stem}_parsed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parse NURA .NLG binary flight logs into readable files.")
    parser.add_argument("input", type=Path, help="Path to a .NLG file.")
    parser.add_argument(
        "-o",
        "--out-dir",
        type=Path,
        help="Output directory. Defaults to <input_stem>_parsed beside the input file.",
    )
    parser.add_argument(
        "--no-split",
        action="store_true",
        help="Only write combined files, not per-record-type CSV files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input
    out_dir = args.out_dir or default_output_dir(input_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    data = input_path.read_bytes()
    frames = parse_nlg(data)
    rows = [row_for_frame(frame) for frame in frames]

    write_csv(out_dir / "frames.csv", rows)
    (out_dir / "frames.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    record_counts: dict[str, int] = {}
    for frame in frames:
        record_counts[frame.record_type] = record_counts.get(frame.record_type, 0) + 1

    summary = {
        "input": str(input_path),
        "input_bytes": len(data),
        "frames": len(frames),
        "first_sequence": frames[0].sequence if frames else None,
        "last_sequence": frames[-1].sequence if frames else None,
        "first_timestamp_ms": frames[0].timestamp_ms if frames else None,
        "last_timestamp_ms": frames[-1].timestamp_ms if frames else None,
        "record_counts": record_counts,
        "outputs": {
            "frames_csv": str(out_dir / "frames.csv"),
            "frames_json": str(out_dir / "frames.json"),
        },
    }

    if not args.no_split:
        for record_type in sorted(record_counts):
            type_rows = [row for row in rows if row["record_type"] == record_type]
            file_name = f"{record_type.lower()}.csv"
            write_csv(out_dir / file_name, type_rows)
            summary["outputs"][file_name.removesuffix(".csv")] = str(out_dir / file_name)

    summary["outputs"]["summary_json"] = str(out_dir / "summary.json")
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"Parsed {len(frames)} frames from {input_path}")
    print(f"Wrote {out_dir}")
    for record_type, count in sorted(record_counts.items()):
        print(f"  {record_type}: {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
