# Secure Uplink Pairing V1

Status: Bench uplink hardening

## Purpose

Reduce the chance that another rocket or ground station operating on the same
legal RF channel can create a command accepted by NURA avionics. This layer does
not replace vehicle ID or SipHash authentication; it adds an authenticated
pairing field that must match before command dispatch.

## Inputs And Units

- NURA V2 Lite CONTROL/CMD uplink frame.
- Authenticated frame tag and authenticated CONTROL tag.
- `vehicle_id`.
- `command_seq` and `nonce`.
- `param1 == 0x4E55`, the NURA pairing magic.

## Allowed States

The pairing check runs before every CONTROL command is dispatched. Bench reset
uplink remains allowed only in the `debug_lora_reset_uplink` bench environment.

## Forbidden States

Any CONTROL command with the wrong pairing magic is rejected in all FSM states.
The command is not remembered as processed and no FSM request is raised.

## Thresholds And Sources

The pairing magic is not a physical threshold. Its source is a team protocol
decision for reducing cross-team command acceptance during shared-band tests.

## Failure Modes Considered

- Same channel, different rocket: rejected by `vehicle_id` and authenticated
  pairing.
- Same vehicle ID but wrong command formatter: rejected by CONTROL auth tag or
  pairing magic.
- Replayed old packet: still covered by command sequence and nonce duplicate
  cache.
- Bench reset packet without pairing field: rejected before reset request is
  raised.

## Fallback Behavior

If GCS and avionics pairing constants do not match, commands time out or return
`REJECT_PROFILE_REJECTED`; telemetry downlink continues.

## Verification Plan

- Build `main` and `debug_lora_reset_uplink`.
- Run FSM replay tests.
- Run GCS hardware-flow and mission-control contract tests.
- Bench-test reset uplink and confirm `ACCEPTED -> EXECUTED` with matched GCS.
