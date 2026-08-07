# Authenticated ARM Uplink and SAFE-to-ARMED Transition

Status: implemented for software integration; radio-pair hardware verification is still required before flight.

## Purpose

The production flight controller must remain in `SAFE` after initialization until
the ground station sends an authenticated, fresh `ARM_FLIGHT` command. Packet
parsing never writes the flight state directly. `TelemetryTask` validates and
queues the request, and `FlightStateMachineTask` remains the sole owner of the
`SAFE -> ARMED` transition.

ARM is a state-transition request only. It does not energize drogue or main pyro
outputs and does not relax abort, sensor, continuity, battery, or physical arming
interlocks.

## Wire Contract

The command uses the existing fixed 24-byte CONTROL payload and 43-byte frame.
No packet length or authentication input changes.

| CONTROL offset | Size | ARM value | Meaning |
| ---: | ---: | ---: | --- |
| 0 | 1 | `0x01` | `CONTROL_CMD` |
| 1 | 1 | `0x04` | `COMMAND_ARM_FLIGHT` |
| 2 | 2 | variable LE | Logical command sequence |
| 4 | 4 | variable LE | Nonce |
| 8 | 4 | non-zero LE | Avionics `millis()` expiry, in ms |
| 12 | 2 | `1` signed LE | Expected source state, `FLIGHT_SAFE` |
| 14 | 2 | `2` signed LE | Requested target state, `FLIGHT_ARMED` |
| 16 | 8 | variable | Existing CONTROL SipHash tag |

The frame direction domain is `UPLINK(0x55)`. Frame CRC, frame authentication,
and CONTROL authentication use the existing NURA V2 Lite rules.

Public bench-key golden vector:

```text
frame_seq     = 0x1234
command_seq   = 0x3344
nonce         = 0xA1B2C3D4
validUntilMs  = 54321

aa 55 23 41 52 55 4e 34 12 01 04 44 33 d4 c3 b2
a1 31 d4 00 00 01 00 02 00 cf 9d 6d 96 41 a0 11
0a 13 d5 1b 51 d9 de 6e cd d2 3d
```

## Inputs And Sources

| Input | Unit | Required value | Source |
| --- | --- | --- | --- |
| Frame/control auth | bytes | Existing tags must verify | NURA V2 Lite protocol |
| `validUntilMs` | ms since avionics boot | Non-zero and not expired | GCS handoff contract; GCS uses latest FAST boot time plus 3000 ms |
| `param0` | state ID | `FLIGHT_SAFE(1)` | Stable state enum/wire contract |
| `param1` | state ID | `FLIGHT_ARMED(2)` | Stable state enum/wire contract |
| Current state | enum | Exactly `State::SAFE` | Shared `FlightState`, owned by FSM |
| Abort status | boolean | Inactive | Shared `AbortState`, owned by safety/watchdog path |
| Pending transition | boolean/seq | No ARM or bench-reset request/ACK pending | Shared request state and telemetry ACK state |

There is no sensor threshold in this decision. The 3000 ms GCS validity window
is a team interface decision, not a sensor-derived flight threshold. Avionics
only enforces the transmitted non-zero deadline using wrap-safe signed time
comparison.

## Processing And State Ownership

```text
LoRa RX
  -> frame length, CRC, direction, vehicle ID, frame auth
  -> CONTROL decode and CONTROL auth
  -> expiry and duplicate checks
  -> ARM format/state/abort/pending checks
  -> queue FlightState.armRequested and ACK_ACCEPTED
  -> FSM consumes request on its next tick
  -> transitionTo(ARMED), including onEnter(ARMED)
  -> record armExecuted and matching sequence
  -> queue ACK_EXECUTED with state_after=FLIGHT_ARMED
```

`ACK_EXECUTED/RESULT_OK` is allowed only when the matching request sequence has
been executed and the current state is actually `ARMED`. A duplicate command is
not re-queued; it receives `ACK_DUPLICATE`, while the original deferred EXECUTED
ACK remains pending.

## Allowed And Forbidden States

| Current state | Result |
| --- | --- |
| `SAFE`, abort inactive, no transition pending | Request accepted |
| `INIT`, `ARMED`, `LAUNCH`, `COAST`, `APOGEE`, `DROGUE`, `DEPLOY`, `GROUND`, `FAULT` | `ACK_REJECTED / RESULT_BAD_STATE / REJECT_STATE_REJECTED` |
| `SAFE` with abort active | Same BAD_STATE rejection |

Production `SAFE` has no automatic transition or timeout fallback. If ARM is
rejected, the controller remains in `SAFE`; the operator must clear the cause
and issue a new authenticated command with a new valid deadline. Bench-only
`NURA_BENCH_FSM_AUTOFLOW` and serial-step builds retain their explicitly gated
test transitions.

## Failure Modes

| Failure | Behavior |
| --- | --- |
| CRC/frame auth invalid | Drop frame; do not trust command identity enough to ACK |
| CONTROL auth invalid | Reject with `AUTH_FAILED` |
| Deadline expired | Reject with `EXPIRED` |
| Deadline zero or state parameters wrong | Reject with `BAD_FORMAT` |
| State not SAFE, abort active, or request conflict | Reject with `BAD_STATE` |
| Abort becomes active after ACCEPTED but before FSM consumption | Cancel request, remain SAFE, send deferred `REJECTED/BAD_STATE`; never execute later |
| Exact command retransmitted | Send DUPLICATE; do not re-execute or erase original pending EXECUTED ACK |
| ACK queue temporarily full | Keep deferred completion pending and retry on later telemetry ticks |
| Radio unavailable | State cannot be remotely armed; production remains SAFE |

## Pyro Invariant

Neither ARM packet handling nor `onEnter(State::ARMED)` calls `setDrogue()`,
`setMain()`, or changes pyro GPIO. The FSM replay test records the pyro HAL calls
and requires zero ON calls during both successful ARM and abort-race rejection.
Physical arming hardware remains an independent energetic inhibit.

## Implementation Map

| Responsibility | File |
| --- | --- |
| Command ID and wire types | `protocol/include/nura_protocol_v1_lite.h` |
| ARM validation policy | `src/missions/telemetry/arm_command_policy.h` |
| Authentication, deduplication, queueing, ACKs | `src/missions/telemetry/telemetry_task.cpp` |
| Request/execution/rejection handshake | `src/state/flight_state.h` |
| Sole state transition and abort-race cancellation | `src/missions/flight/fsm_task.cpp` |
| Production RX intent | `include/nura_constants.h` (`kFlightDownlinkOnly=false`) |

## Verification

Board-free acceptance checks:

```bash
python3 test/protocol/run_protocol_tests.py
python3 test/telemetry/run_arm_uplink_tests.py
pio run -e main
pio run -e debug
```

The protocol test decodes and re-encodes the golden vector byte-for-byte. ARM
policy tests cover zero expiry, incorrect source/target, all non-SAFE states,
abort, and pending-request conflicts. The host telemetry flow test executes the
real `TelemetryTask` and FSM with a fake radio to verify ACCEPTED, eight exact
retries as DUPLICATE, deferred EXECUTED, abort-race REJECTED, one transition,
no pyro energization, and FAST/GPS resumption after ACK traffic.

Required hardware integration before flight:

1. Confirm FAST remains periodic while the radio returns to RX after every TX.
2. Send the golden-equivalent provisioned ARM command from the actual GCS.
3. Observe `ACK_ACCEPTED(SAFE)` followed by `ACK_EXECUTED/OK(ARMED=2)`.
4. Retransmit the same frame eight times at 250 ms and verify one transition.
5. Repeat with abort active and verify SAFE plus REJECTED, with pyro outputs low.
