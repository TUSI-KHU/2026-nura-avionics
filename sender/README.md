# NURA LoRa V2 Sender Bench Emulator

This folder is reserved for the avionics-side LoRa sender implementation.

The implementation follows `documents/nura_lora_packet_protocol_v1.md` and the shared protocol header in `protocol/include/nura_protocol_v1_lite.h`.

Current firmware behavior:

1. Sends synthetic FAST_TLM at 5 Hz.
2. Sends synthetic GPS_TLM at 1 Hz.
3. Receives CONTROL/CMD.
4. Authenticates and deduplicates commands.
5. Sends CONTROL/ACK for FORCE_DEPLOY_RECOVERY and ABORT_PROPULSION_DEPRECATED.
6. Skips telemetry whenever ACK has priority.

Do not transmit raw C++ structs directly. Encode every integer field explicitly as little-endian bytes.

Build with:

```sh
pio run -d sender
```
