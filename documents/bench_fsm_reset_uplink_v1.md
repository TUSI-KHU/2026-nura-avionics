# Bench FSM Reset Uplink V1

Status: Bench-only test feature
Target: GCS integration and state-machine bench testing

## Purpose

Allow the GCS reset control to request a bench-only reset of the avionics flight
state machine after a desktop or benchtop test has driven the FSM into recovery
states.

## Inputs And Units

- Authenticated NURA V2 Lite CONTROL uplink frame.
- `COMMAND_BENCH_RESET_FSM` command ID.
- `command_seq` and `nonce` for duplicate detection.
- `param0 == 0`.
- `param1 == 0x4E55`, the authenticated NURA pairing magic.
- System time in milliseconds.

## Allowed States

The bench reset request is accepted in any FSM state only when the firmware is
built with `NURA_ENABLE_BENCH_FSM_RESET_UPLINK`. The provided
`debug_lora_reset_uplink` bench environment also defines
`NURA_DISABLE_PYRO_OUTPUTS`, so the MOSFET pyro HAL leaves GPIO outputs disabled.
That same flag reports pyro output as not implemented, so FORCE_DEPLOY is not an
available command in this bench environment.

## Forbidden States

All states forbid this command in normal flight builds. Without
`NURA_ENABLE_BENCH_FSM_RESET_UPLINK`, the telemetry task rejects the command with
`RESULT_NOT_SUPPORTED`.

## Behavior

The telemetry task authenticates the command and marks a reset request on
`FlightState`. The FSM task consumes the request, turns all pyro outputs off,
clears FSM scratch state and trace buffers, and returns the public state to
`INIT`. Normal FSM ticks then proceed through the existing `INIT -> SAFE ->
ARMED` path if no abort is active.

## Thresholds And Sources

No new physical threshold is introduced. The command is gated only by the
bench-only build flag and authenticated command framing.

## Failure Modes Considered

- Flight build receives reset command: rejected as not supported.
- Malformed params: rejected as bad format or profile rejected.
- Duplicate command: handled by the existing recent-command cache.
- Reset from recovery state: pyro outputs are forced off before state is reset.
- Bench reset firmware accidentally used with pyro connected: the provided bench
  environment disables the MOSFET pyro output driver in software, but physical
  pyro separation is still required for bench testing.

## Fallback Behavior

If the command is unavailable or the uplink does not respond, bench operators
must reset the avionics by USB reboot, physical reset, power cycle, or firmware
upload.

## Verification Plan

- Run FSM replay tests to confirm existing transitions still pass.
- Build the normal `main` environment and the `debug_lora_reset_uplink` bench
  environment.
- On bench hardware with pyro physically disconnected, start GCS and verify that
  RESET VIEW clears the UI while RESET AVIONICS returns telemetry state to the
  initial `INIT -> SAFE -> ARMED` sequence.
