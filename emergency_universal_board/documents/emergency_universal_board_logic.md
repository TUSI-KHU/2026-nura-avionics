# Emergency Universal Board Logic

## Purpose

Run the existing NURA flight state machine on a hand-wired Teensy 4.1 fallback board using only BMP390, BNO085, SD, QSPI flash, and two active-high MOSFET outputs.

## Inputs And Units

- BNO085 acceleration: m/s^2, copied to `ImuState` and converted to g in `HighGImuState`.
- BNO085 gyroscope: deg/s.
- BNO085 orientation: roll/pitch/yaw in degrees.
- BMP390 pressure: Pa.
- BMP390 altitude: meters AGL relative to boot-time reference pressure.
- Pyro outputs: Teensy digital pins 2 and 3, active-high.

## Allowed States

Sensor sampling and logging run from boot after scheduler initialization. Pyro output is still controlled only by the existing flight state machine.

## Forbidden States

Packet parsing, telemetry reception, raw sensor parsing, and direct variable assignment must not energize pyro outputs. LoRa/uplink recovery is disabled in this fork.

## Thresholds

The same FSM thresholds remain in `include/nura_constants.h`. No new launch, burnout, apogee, main, or landing thresholds were invented for this emergency fork.

## Failure Modes Considered

- Missing BMP390: barometer task retries and marks barometer data invalid.
- Missing BNO085: critical recoverable task initialization fails and panic path is reached.
- Missing SD: existing mirrored storage behavior keeps QSPI as the other storage path where possible.
- Pyro pin conflict: voltage-sense pin is unassigned and ignored by the MOSFET conflict check.
- BNO085 high-acceleration inadequacy: documented as a same-day bench-verification risk.

## Fallback Behavior

The existing FSM fallback behavior is preserved. The BNO085 is published into both low-g and high-g state structures so existing launch/burnout acceleration consumers do not need a new state-machine path.

## Verification Plan

- Build: `pio run -d emergency_universal_board -e main`.
- Host FSM replay: `python3 test/fsm_replay/run_fsm_replay_tests.py`.
- OpenRocket-derived nominal replay must pass INIT-to-GROUND ordering with apogee decision no more than 10 m below simulated apogee.
- Sensor bring-up: serial logs must include `bno085 initialized` and `bmp390 initialized`.
- Storage: inspect created flight log segments on SD and confirm QSPI logging does not panic.
- Pyro: run a continuity-safe bench test with no charges connected and verify pins 2/3 remain LOW until commanded by a test flow.
- Flight-readiness: replay or bench-motion test must show launch threshold confirmation before any live flight use.

## Latest Host Replay Result

OpenRocket-derived nominal sustainer input, using `502.911 m` apogee at `10.500 s`, reached:

- SAFE at `0 ms`
- ARMED at `10 ms`
- LAUNCH at `1030 ms`
- COAST at `2580 ms`
- APOGEE at `10050 ms`
- DROGUE at `13050 ms`
- DEPLOY at `22800 ms`
- GROUND at `31800 ms`
