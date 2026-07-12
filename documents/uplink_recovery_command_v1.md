# Uplink Recovery Command V1

## Purpose

The avionics accepts an authenticated `FORCE_DEPLOY_RECOVERY` uplink without
allowing radio parsing to energize a pyro output or assign a flight state.

## Inputs And Units

- One SX1276 LoRa PHY packet, in bytes.
- One exact 43-byte authenticated NURA V2 Lite CONTROL frame per PHY packet.
- Frame sync, protocol version, message type, CRC-16, command authentication
  tag, command sequence, nonce, and `valid_until_ms` in avionics boot
  milliseconds.
- Current `FlightState::state` from the flight state machine.
- `param0`; only value `0` is supported for the current recovery channel.

## Allowed States

The command may create a recovery request only in `LAUNCH` or `COAST`. The
flight state machine checks the state again when consuming that request. This
is the team-selected conservative interpretation of "after launch": once the
normal recovery sequence is active, a repeated command is acknowledged as
already done and cannot retrigger it.

## Forbidden States

`INIT`, `SAFE`, `ARMED`, `APOGEE`, `DROGUE`, `DEPLOY`, `GROUND`, and `FAULT`
must not start a new forced recovery sequence. Packet parsing and telemetry
task code must never drive a pyro output directly.

The current build also rejects every forced recovery command with
`ACTUATOR_FAULT/DEPLOYMENT_INHIBITED` because no pyro GPIO, arming input,
continuity input, or battery gate is assigned in `board_pinmap.h`. This inhibit
must remain until the reviewed hardware mapping and inert-load verification are
implemented.

## Thresholds And Sources

- CONTROL payload length 24 bytes and frame length 43 bytes: NURA V2 Lite
  protocol definition in `documents/nura_lora_packet_protocol_v1.md`.
- CRC-16/CCITT-FALSE parameters and SipHash-2-4 authentication tag: the same
  protocol definition.
- Allowed `LAUNCH` and `COAST` states: team requirement that uplink deployment
  is inhibited before launch, implemented conservatively to avoid retriggering
  an active recovery sequence.
- RF frequency 920.9 MHz for the default flight build: current team firmware
  configuration. Legal frequency, power, antenna, and range remain subject to
  ground and integration test approval.

Without `include/nura_radio_secrets.h`, the build uses a public test identity and
key and is unsafe for flight. The gitignored provisioning header must contain a
team-assigned vehicle ID and secret key before flight acceptance.

## Failure Modes And Fallback

- Oversized packets are drained from the SX1276 FIFO without writing beyond
  the caller buffer, then rejected.
- Short, long, concatenated, wrong-sync, wrong-version, wrong-type, and bad-CRC
  packets are rejected before command decoding.
- Bad authentication, expired commands, unsupported parameters, and forbidden
  states are rejected with an ACK when queue capacity permits.
- Duplicate commands do not execute twice.
- A full ACK queue drops the new ACK and logs the condition; it does not block
  the scheduler or execute a command through another path.
- Radio or ACK failure does not bypass the state-machine request guard. Normal
  autonomous recovery logic remains the fallback.
- Missing actuator hardware returns a rejected ACK; it must never return
  `EXECUTED` for a state transition without electrical actuation feedback.

## Verification Plan

1. Run host protocol tests for exact length, trailing data, sync, version, and
   CRC rejection.
2. Run FSM replay tests for accepted `COAST` and rejected pre-launch requests.
3. Build the default SX1276 flight configuration and the explicit SX1278 bench
   configuration.
4. Before flight, perform two-radio hardware integration tests for malformed
   packets, duplicate/expired/authentication failures, ACK queue pressure, and
   RF range.
5. Bench-test the recovery controller with inert loads before any pyro hardware
   integration. No flight acceptance is permitted with the public test key.
