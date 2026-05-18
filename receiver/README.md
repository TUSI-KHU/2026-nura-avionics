# NURA LoRa Receiver

This folder is reserved for the ground-side receiver and GCS protocol implementation.

The implementation follows `documents/nura_lora_packet_protocol_v1.md` and the shared protocol header in `protocol/include/nura_protocol_v1_lite.h`.

Current firmware behavior:

1. Decodes FAST_TLM, GPS_TLM, and CONTROL.
2. Sends CONTROL/CMD FORCE_DEPLOY_RECOVERY after receiving enough telemetry.
3. Retries until CONTROL/ACK EXECUTED.
4. Sends CONTROL/CMD ABORT_PROPULSION_DEPRECATED and expects REJECTED/NOT_SUPPORTED.
5. Prints PASS lines when the V1 Lite pair test completes.

The Arduino gateway must remain a dumb raw-byte bridge. All decoding and command logic belongs here or in the GCS process.

Build with:

```sh
pio run -d receiver
```

Two-board protocol test:

```sh
python3 receiver/tools/run_pair_test.py --duration 20
```
