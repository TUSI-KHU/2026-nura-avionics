# Bench No-Uplink RX-Off Profile V1

Status: Bench-only no-uplink radio profile

## Purpose

Provide an avionics bench firmware profile that transmits LoRa telemetry while
keeping the SX1276 uplink receive path disabled. This reduces the chance that a
nearby rocket or ground station can put a command frame into the avionics command
decoder during bench interoperability tests.

## Inputs And Units

- LoRa RF profile: `922300000 Hz`, SF7, BW125, CR4/5, sync word `0x12`.
- Radio TX drive: `2 dBm`.
- `Sx127xLoRaConfig::downlinkOnly`.

## Allowed States

The profile may run in bench tests only when energetic devices are physically
separated. It is represented by the `debug_lora_no_uplink` PlatformIO
environment.

## Forbidden States

Uplink command reception is forbidden in every FSM state for this profile.
`NURA_BENCH_ENABLE_UPLINK` and `NURA_ENABLE_BENCH_FSM_RESET_UPLINK` are not
defined, and the SX1276 HAL leaves the receive path disabled whenever
`downlinkOnly` is true.

## Behavior

The radio may transmit telemetry. After initialization and each synchronous
transmit attempt, the SX1276 RF switch is left with RXE and TXE disabled instead
of returning to receive mode. Calls to `receive()` return false before
`LoRa.parsePacket()` when `downlinkOnly` is true.

## Thresholds And Sources

No physical flight threshold is introduced. The no-uplink gate is a team safety
and interference-resilience decision.

## Failure Modes Considered

- Nearby LoRa command-like packets: RX parsing is disabled in the avionics HAL.
- GCS reset button pressed against this firmware: no avionics ACK is expected;
  operators must use USB reset, physical reset, power cycle, or a different
  bench firmware when reset-uplink testing is needed.
- Accidental use with pyro connected: this bench environment also defines
  `NURA_DISABLE_PYRO_OUTPUTS`, but physical separation is still required.

## Fallback Behavior

Use the normal `main` build for flight-like downlink-only checks, or use
`debug_lora_reset_uplink` only when bench reset uplink is intentionally required.

## Verification Plan

- Build `main`, `debug_lora_no_uplink`, and `debug_lora_reset_uplink`.
- Run FSM replay tests.
- On bench hardware, upload `debug_lora_no_uplink` and confirm telemetry is
  received while GCS reset-uplink commands time out without avionics ACK.
