# NURA LoRa Receiver

Ground-side Teensy 4.1 receiver for NURA V1 Lite telemetry.

The implementation follows `documents/nura_lora_packet_protocol_v1.md` and the shared protocol header in `protocol/include/nura_protocol_v1_lite.h`.

Default firmware behavior:

1. Decodes FAST_TLM, GPS_TLM, and CONTROL/ACK frames.
2. Prints real avionics sensor telemetry with engineering units:
   - FAST: flight state, status word, pressure delta, low-g acceleration, gyro, battery, health bits, RSSI/SNR.
   - GPS: fix, latitude/longitude, altitude, speed, course, HDOP, satellites, age, RSSI/SNR.
3. Does not transmit recovery/control commands.

Pair-test-only behavior is available in the `pair_test` environment. That build defines `NURA_RECEIVER_AUTO_COMMAND_TEST` and automatically sends FORCE_DEPLOY_RECOVERY followed by the deprecated abort command check. Do not use that environment with flight hardware unless the recovery output path is intentionally inhibited for a bench test.

The Arduino gateway must remain a dumb raw-byte bridge. All decoding and command logic belongs here or in the GCS process.

Build receive-only ground station firmware with:

```sh
pio run -d receiver -e teensy41
```

Upload and monitor receive-only firmware with:

```sh
pio run -d receiver -e teensy41 -t upload
pio device monitor -d receiver -b 115200
```

Two-board protocol test with automatic bench commands:

```sh
python3 receiver/tools/run_pair_test.py --duration 20
```
