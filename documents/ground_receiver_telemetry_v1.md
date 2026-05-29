# Ground Receiver Telemetry and Bench Command Gate V1

## Purpose

The ground-side Teensy receiver decodes NURA V1 Lite LoRa frames from the avionics radio and prints engineering-unit telemetry for GCS or serial-monitor use.

The default receiver firmware is receive-only. It must not transmit recovery/control commands when connected to real avionics hardware.

A separate `pair_test` PlatformIO environment enables automatic CONTROL/CMD transmission for the two-board bench protocol test only.

## Inputs and Units

FAST_TLM inputs from avionics:

| Field | Unit |
| --- | --- |
| `status_word` | bitfield defined in `nura_lora_packet_protocol_v1.md` |
| `boot_ms` | ms since avionics boot |
| `baro_dp_2pa` | pressure delta / 2 Pa |
| `low_accel_*_cg` | g * 100 |
| `gyro_*_dps10` | deg/s * 10 |
| `batt_mv` | mV |

GPS_TLM inputs from avionics:

| Field | Unit |
| --- | --- |
| `latitude_e7`, `longitude_e7` | degrees * 10,000,000 |
| `gps_alt_dm` | m * 10 |
| `speed_cms` | m/s * 100 |
| `course_cdeg` | degrees * 100 |
| `hdop_x10` | HDOP * 10 |
| `satellites` | count |
| `age_100ms` | 100 ms |

CONTROL/ACK inputs are decoded for operator visibility and pair-test validation.

## Allowed States

Receive-only mode is allowed for lab, integration, pad, and flight-rehearsal telemetry observation, subject to local radio legality and team operating procedures.

`pair_test` automatic command mode is allowed only when all of the following are true:

1. The setup is a bench protocol test.
2. The recovery output path is inhibited, disconnected, or otherwise intentionally made safe.
3. The firmware was explicitly built with `NURA_RECEIVER_AUTO_COMMAND_TEST` through `pio run -d receiver -e pair_test` or `receiver/tools/run_pair_test.py`.

## Forbidden States

`pair_test` automatic command mode is forbidden with flight-ready recovery hardware connected, with live energetic devices connected, or during any pad/flight operation.

The receive-only `teensy41` environment must remain the default build for real avionics telemetry checks.

## Thresholds and Sources

The command gate has no physical threshold. Its source is a team safety decision: automatic FORCE_DEPLOY_RECOVERY transmission is a bench-test feature and must be behind an explicit build flag.

The telemetry scaling thresholds and units come from `documents/nura_lora_packet_protocol_v1.md` and `protocol/include/nura_protocol_v1_lite.h`.

## Failure Modes Considered

- Operator uploads pair-test receiver firmware to real hardware by mistake.
- GCS emits FORCE_DEPLOY_RECOVERY after merely receiving telemetry.
- Serial output hides real sensor values, making hardware integration failures hard to identify.
- Pair-test automation breaks when the default receiver is made receive-only.

## Fallback Behavior

If `NURA_RECEIVER_AUTO_COMMAND_TEST` is not defined, `serviceCommandSender()` and pair-test completion printing return without transmitting commands.

Telemetry decode and CONTROL/ACK visibility remain active in both receiver environments.

## Verification Plan

- Build receive-only receiver: `pio run -d receiver -e teensy41`.
- Build pair-test receiver: `pio run -d receiver -e pair_test`.
- Run script syntax check: `python3 -m py_compile receiver/tools/run_pair_test.py`.
- Hardware bench verification: upload real avionics firmware and receive-only receiver, then confirm FAST/GPS lines show changing IMU/barometer/GNSS fields and no `cmd tx` lines appear.
- Pair-test verification: run `python3 receiver/tools/run_pair_test.py --duration 20` with inhibited recovery outputs and confirm FAST/GPS/CONTROL pass criteria.
