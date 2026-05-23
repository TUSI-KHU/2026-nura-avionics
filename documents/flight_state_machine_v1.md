# NURA Flight State Machine V1

Status: Draft for first FSM implementation  
Target vehicle: 2026 NURA avionics  
Target controller: Teensy 4.1  

This document defines the currently agreed first implementation of the NURA
flight state machine. It intentionally excludes high-rate logging and detailed
sensor fault policy; those are tracked as TODO items for later work.

## Scope

Included in this version:

- State names and sequential enum order.
- Entry actions.
- Periodic actions.
- Timeouts and fallback transitions.
- Transition conditions from the previous state.
- Real-time apogee prediction using a nine-sample quadratic fit.
- Uplink forced recovery deployment as an FSM-owned request path.
- Constants that must be represented in code, even if their final values are
  still team-tunable.

Excluded from this version:

- SD/SPI Flash logging behavior, except TODO notes.
- Detailed sensor fault policy.
- Hardware-in-the-loop replay with real recorded flight logs.

## Global Operating Assumptions

- After `INIT`, sensor acquisition tasks run continuously in every state.
- The FSM reads the latest shared sensor/state values; it does not directly
  perform sensor bus reads.
- Logging is not part of the current FSM implementation. Add TODO markers where
  logging events should be emitted later.
- Sensor fault fallback is not part of the current FSM implementation.
- Constants and thresholds must live in code as named constants, not inline
  literals.
- `APOGEE` is a real state in this design. It represents the drogue pyro
  sequence state, not merely an instantaneous event.

## Enum Order

State IDs are sequential and must remain stable once telemetry or log parsing
depends on them.

| ID | Enum name | Meaning |
| ---: | --- | --- |
| 0 | `INIT` | Boot, sensor initialization, and calibration |
| 1 | `SAFE` | Powered but inactive safety state |
| 2 | `ARMED` | Armed, waiting for launch detection |
| 3 | `LAUNCH` | Launch detected, motor burn / high-acceleration phase |
| 4 | `COAST` | Burnout detected, coasting toward apogee |
| 5 | `APOGEE` | Apogee decision reached; drogue pyro sequence active |
| 6 | `DROGUE` | Drogue sequence complete; waiting for main deploy condition |
| 7 | `DEPLOY` | Main pyro sequence active / main deployed transition state |
| 8 | `GROUND` | Flight complete / recovered or assumed landed |
| 9 | `FAULT` | Fault hold state |

## Constants

Initial implementation constants:

| Constant | Initial value | Unit | Source / note |
| --- | ---: | --- | --- |
| `LAUNCH_ACCEL_THRESHOLD_G` | `2.0` | g | Team decision, based on 2025 acceleration-norm logs |
| `LAUNCH_CONFIRM_SAMPLES` | `4` | samples | Team decision; high-g consecutive samples |
| `BURNOUT_ACCEL_THRESHOLD_G` | `1.0` | g | Team decision; high-g acceleration norm below this enters coast |
| `BURNOUT_CONFIRM_SAMPLES` | `4` | samples | Team decision; high-g consecutive samples |
| `APOGEE_FIT_WINDOW_SAMPLES` | `9` | samples | Barometer sliding-window quadratic fit |
| `APOGEE_PREDICTION_HISTORY_SAMPLES` | `5` | samples | Recent valid apogee predictions used by `plus2sigma5` |
| `APOGEE_CONFIRM_SAMPLES` | `3` | samples | Consecutive valid apogee predictions before deployment |
| `APOGEE_MIN_FLIGHT_TIME_MS` | `8000` | ms | Reject prediction and descent backup before this flight time |
| `APOGEE_MAX_PREDICT_AHEAD_S` | `1.0` | s | Reject apogee predictions too far in the future |
| `APOGEE_DEPLOY_ALT_MARGIN_M` | `3.0` | m | Trigger when predicted apogee is this close |
| `APOGEE_MAX_ALT_MARGIN_M` | `20.0` | m | Reject predictions too far above current altitude |
| `APOGEE_MIN_CURVATURE` | `0.05` | m/s^2-scale | Reject near-linear fits; fit coefficient `a` must be `< -0.05` |
| `APOGEE_MAX_CURVATURE` | `120.0` | m/s^2-scale | Reject unrealistically sharp fitted curves; fit coefficient `a` must be `> -120` |
| `APOGEE_MAX_FIT_RMSE_M` | `2.5` | m | Reject noisy nine-sample quadratic fits |
| `APOGEE_MAX_PREDICTION_JUMP_M` | `15.0` | m | Reject a new raw prediction that jumps too far from the last accepted raw prediction |
| `APOGEE_MAX_PREDICTION_SIGMA_M` | `8.0` | m | Reject unstable five-prediction history |
| `APOGEE_AGGREGATION_SIGMA_MULTIPLIER` | `2.0` | sigma | Use `mean + 2*sigma` over the latest five valid raw apogee predictions |
| `APOGEE_MAX_BARO_SAMPLE_GAP_MS` | `150` | ms | Reset apogee fit history after a stale barometer gap |
| `MIN_APOGEE_DETECT_ALT_M` | `30.0` | m AGL | Prevent low-altitude false deployment |
| `APOGEE_DROP_THRESHOLD_M` | `4.0` | m | Backup descent detector |
| `APOGEE_DESCENT_CONFIRM_SAMPLES` | `4` | samples | Consecutive backup descent samples |
| `BARO_MEDIAN_WINDOW_SAMPLES` | `3` | samples | Spike rejection in barometer task; implementation-local constant |
| `BARO_ALTITUDE_LPF_ALPHA` | `0.35` | ratio | First-order low-pass on median altitude; implementation-local constant, tune after replay |
| `PYRO_FIRE_DURATION_MS` | `50` | ms | Initial constant, must be verified by ground test |
| `DROGUE_BACKUP_DELAY_MS` | `2000` | ms | Team decision; delay between primary and backup drogue pyro |
| `MAIN_DEPLOY_ALTITUDE_M_AGL` | `200.0` | m | Initial team decision |
| `APOGEE_TIMEOUT_MS` | `12000` | ms | Initial fallback, must be reviewed against simulation / flight data |
| `MAIN_TIMEOUT_MS` | `15000` | ms | Initial fallback, must be reviewed against descent policy |
| `GROUND_TIMEOUT_MS` | `60000` | ms | Initial placeholder for later landing / recovery policy |

Notes:

- The 4-sample launch and burnout confirmations are intentionally sample-count
  based, not millisecond based.
- The launch and burnout confirmation source is the high-g accelerometer
  (`H3LIS331DL`) acceleration norm.
- Low-g IMU fallback is deferred to the later sensor fault policy.

## State Table

| State | Entry action | Periodic action | Timeout | Fallback | Entry condition from previous state |
| --- | --- | --- | --- | --- | --- |
| `INIT` | Initialize sensors, initialize calibration, build barometer ground baseline, force pyro outputs OFF. TODO: initialize SD/SPI Flash logging later. | Check initialization completion. | TODO init timeout. | `FAULT` or `SAFE`, final policy TBD. | Boot starts here. |
| `SAFE` | Force pyro outputs OFF. Reset flight-local counters and detector state. | Watch arming switch / arming command. Sensor tasks continue globally. | None for first implementation. | None. | `INIT` completed. |
| `ARMED` | Reset launch detector consecutive count. Reset launch timestamp. Reset coast/apogee detector scratch state. | Compute high-g acceleration norm and update launch confirmation count. | Optional arming timeout TODO. | `SAFE` if disarmed, final disarm policy TBD. | Human arming switch / arming command is active. |
| `LAUNCH` | Save `launch_time_ms`. Reset burnout confirmation count. TODO: log launch event later. | Continue high-g acceleration norm monitoring and update burnout confirmation count. | Optional motor-burn timeout TODO. | `COAST` when motor-burn timeout policy is defined. | In `ARMED`, high-g acceleration norm is `>= LAUNCH_ACCEL_THRESHOLD_G` for `LAUNCH_CONFIRM_SAMPLES` consecutive samples. |
| `COAST` | Save `coast_start_ms`. Reset apogee detector scratch state, including max altitude and descent counter. | Push fresh barometer AGL samples into the nine-sample predictor, update predicted-apogee and backup-descent confirmation counts. | `APOGEE_TIMEOUT_MS`. | `APOGEE`. | In `LAUNCH`, high-g acceleration norm is `< BURNOUT_ACCEL_THRESHOLD_G` for `BURNOUT_CONFIRM_SAMPLES` consecutive samples. |
| `APOGEE` | Start drogue primary pyro pulse. Save pyro sequence start time. TODO: log drogue sequence start later. | End primary pulse after `PYRO_FIRE_DURATION_MS`; after `DROGUE_BACKUP_DELAY_MS`, fire backup pyro for `PYRO_FIRE_DURATION_MS`; keep all pyro timing non-blocking. | Pyro sequence timeout derived from backup delay plus pulse duration. | `DROGUE`. | Apogee detector asserts apogee, or `COAST` timeout fallback fires. |
| `DROGUE` | Latch drogue sequence complete. Reset main deploy detector scratch state. | Monitor barometer AGL altitude for main deployment threshold. | `MAIN_TIMEOUT_MS`. | `DEPLOY`. | Drogue primary-plus-backup pyro sequence is complete. This does not require proof that the parachute physically opened. |
| `DEPLOY` | Start main pyro pulse. TODO: support backup main pyro only if hardware/team policy requires it. TODO: log main deploy later. | End main pulse after `PYRO_FIRE_DURATION_MS`; keep timing non-blocking. | Main pyro sequence timeout. | `GROUND` when timeout policy is defined. | In `DROGUE`, altitude AGL is `<= MAIN_DEPLOY_ALTITUDE_M_AGL`, or `MAIN_TIMEOUT_MS` fallback fires. |
| `GROUND` | Force pyro outputs OFF. TODO: close/flush logs and enter recovery telemetry mode later. | TODO: recovery GPS/telemetry behavior. | None. | None. | Main deploy sequence complete, landing detector later, or `GROUND_TIMEOUT_MS` fallback later. |
| `FAULT` | Force pyro outputs OFF. Set fault flag. TODO: log fault later. | Hold fault state. | None. | None. | Initialization failure or future critical fault policy. |

## Detector Details

### High-G Acceleration Norm

Use high-g accelerometer values:

```text
accel_norm_g = sqrt(x_g^2 + y_g^2 + z_g^2)
```

If values are available only in m/s^2:

```text
accel_norm_g = sqrt(ax^2 + ay^2 + az^2) / 9.80665
```

### Launch Detection

Allowed only in `ARMED`.

```text
if accel_norm_g >= LAUNCH_ACCEL_THRESHOLD_G:
    launch_confirm_count += 1
else:
    launch_confirm_count = 0

if launch_confirm_count >= LAUNCH_CONFIRM_SAMPLES:
    transition to LAUNCH
```

### Burnout / Coast Detection

Allowed only in `LAUNCH`.

```text
if accel_norm_g < BURNOUT_ACCEL_THRESHOLD_G:
    burnout_confirm_count += 1
else:
    burnout_confirm_count = 0

if burnout_confirm_count >= BURNOUT_CONFIRM_SAMPLES:
    transition to COAST
```

No launch-to-coast inhibit is used in the first implementation. The team
decision is to use the four-sample confirmation directly.

### Apogee Detection

Allowed only in `COAST`.

First implementation uses these inputs:

- Filtered barometer AGL altitude.
- Max altitude seen since `COAST` entry.
- Coast elapsed time.

Explicit vertical-velocity estimation is a TODO item. The first implementation
fits against the filtered barometer AGL altitude value published by the
barometer task.

#### Barometer altitude filtering

Barometer filtering runs in `BarometerTask`, before the FSM sees the sample.

Raw pressure is converted to pad-relative altitude:

```text
raw_altitude_m = 44330 * (1 - (pressure_pa / reference_pressure_pa)^0.19029495)
```

Then the altitude filter applies:

```text
median_altitude_m = median(latest 3 raw_altitude_m samples)
filtered_altitude_m += BARO_ALTITUDE_LPF_ALPHA *
                       (median_altitude_m - filtered_altitude_m)
```

On the first valid sample after initialization or read failure, the low-pass
state is initialized directly from the current median. This avoids a startup
ramp from zero altitude.

Telemetry field meaning:

| Field | Meaning | Used by flight logic |
| --- | --- | --- |
| `barometer.rawAltitudeM` | Direct pressure-derived AGL altitude before filtering | No |
| `barometer.altitudeM` | Median plus low-pass filtered AGL altitude | Yes |

Apogee prediction and descent backup are ignored until:

```text
launch_elapsed_ms >= APOGEE_MIN_FLIGHT_TIME_MS
```

This suppresses early false apogee decisions during the first eight seconds
after launch.

#### Quadratic apogee prediction

The main apogee detector uses a sliding window of the latest nine filtered
barometric AGL altitude samples in `COAST`.

Fit:

```text
h(t) = a*t^2 + b*t + c
```

Use least-squares over all nine samples. Use relative seconds from the first
sample in the window; do not use raw `millis()` values in the fit.

Prediction:

```text
t_apogee = -b / (2*a)
h_apogee = c - (b*b) / (4*a)
```

The raw quadratic prediction is valid only when all conditions hold:

```text
sample_count == APOGEE_FIT_WINDOW_SAMPLES
a < -APOGEE_MIN_CURVATURE
a > -APOGEE_MAX_CURVATURE
fit_rmse_m <= APOGEE_MAX_FIT_RMSE_M
t_apogee > t_last
t_apogee - t_last <= APOGEE_MAX_PREDICT_AHEAD_S
h_apogee >= current_altitude
h_apogee - current_altitude <= APOGEE_MAX_ALT_MARGIN_M
current_altitude >= MIN_APOGEE_DETECT_ALT_M
launch_elapsed_ms >= APOGEE_MIN_FLIGHT_TIME_MS
abs(h_apogee - previous_raw_h_apogee) <= APOGEE_MAX_PREDICTION_JUMP_M
```

Then the deployment candidate uses the latest five accepted raw apogee
predictions:

```text
mean_h = mean(latest 5 accepted h_apogee values)
sigma_h = population_stddev(latest 5 accepted h_apogee values)
candidate_h = mean_h + 2 * sigma_h
```

This `plus2sigma5` candidate is deliberately conservative. It delays the
deployment trigger compared with the newest raw prediction and reduced early
deployment cases in the OpenRocket perturbation experiment.

The prediction history is valid only when:

```text
accepted_prediction_count >= APOGEE_PREDICTION_HISTORY_SAMPLES
sigma_h <= APOGEE_MAX_PREDICTION_SIGMA_M
```

Deployment trigger:

```text
valid_prediction_history
AND candidate_h - current_altitude >= 0
AND candidate_h - current_altitude <= APOGEE_DEPLOY_ALT_MARGIN_M
for APOGEE_CONFIRM_SAMPLES consecutive barometer samples
```

Quality checks intentionally prefer missing the prediction trigger over firing
too early. If the predictor is rejected, the FSM remains in `COAST` and can
still transition through the descent backup or coast timeout fallback.

#### Backup descent detection

```text
launch_elapsed_ms >= APOGEE_MIN_FLIGHT_TIME_MS
AND max_altitude - current_altitude >= APOGEE_DROP_THRESHOLD_M
for APOGEE_DESCENT_CONFIRM_SAMPLES consecutive barometer samples
```

Required fallback path:

```text
if coast_elapsed_ms >= APOGEE_TIMEOUT_MS:
    transition to APOGEE
```

`APOGEE_TIMEOUT_MS` must not remain unreviewed for flight use and must be
greater than `APOGEE_MIN_FLIGHT_TIME_MS`.

## Pyro Sequence

All pyro control must be non-blocking. Do not use `delay()` for firing pulses.

Drogue sequence in `APOGEE`:

```text
t = apogee_entry_ms

primary ON at t
primary OFF at t + PYRO_FIRE_DURATION_MS

backup ON at t + DROGUE_BACKUP_DELAY_MS
backup OFF at t + DROGUE_BACKUP_DELAY_MS + PYRO_FIRE_DURATION_MS

after backup OFF:
    transition to DROGUE
```

Main sequence in `DEPLOY`:

```text
t = deploy_entry_ms

main ON at t
main OFF at t + PYRO_FIRE_DURATION_MS

after main OFF:
    transition to GROUND or later landing/ground policy state
```

TODO:

- Confirm whether the main channel needs a backup channel and delay.
- Confirm actual e-match / igniter fire duration through ground test.
- Add continuity and battery checks in the later safety policy.

## Uplink Forced Recovery Command

`FORCE_DEPLOY_RECOVERY` is handled as a request into the FSM, not as a direct
state write from telemetry parsing.

Allowed command path:

```text
TelemetryTask validates auth, freshness, format, and coarse state
TelemetryTask sets FlightState.forceRecoveryDeployRequested
FSM consumes the request on its next tick
FSM may transition LAUNCH/COAST -> APOGEE
TelemetryTask sends ACK_EXECUTED only after FSM records execution
```

Allowed states:

| Current state | Command result |
| --- | --- |
| `LAUNCH` | Request accepted; FSM may enter `APOGEE`. |
| `COAST` | Request accepted; FSM may enter `APOGEE`. |
| `APOGEE`, `DROGUE`, `DEPLOY` | Treated as already active / already done. |
| `INIT`, `SAFE`, `ARMED`, `GROUND`, `FAULT` | Rejected by state. |

This keeps packet reception, authentication, and ACK management in telemetry
while keeping the flight-state transition and recovery sequence under the FSM.

## TODO For Later Work

- SD/SPI Flash logging policy and storage buffer sizing.
- Sensor fault policy and degraded-mode transitions.
- Barometer oversampling configuration for 50 ms sampling.
- Barometer filter tuning.
- Landing detector or `GROUND_TIMEOUT_MS` policy.
- Pyro continuity and battery voltage gating.
- Flight-log replay verification tests.

## Implementation Map

This table maps the documented logic to the current code so later reviews can
check implementation drift quickly.

| Logic / data | File | Code location / function | Notes |
| --- | --- | --- | --- |
| State enum and printable names | `src/core/states.h` | `enum class State`, `stateName()` | IDs match the telemetry state code table. |
| Mission thresholds and timeouts | `src/missions/mission_constants.h` | `MissionConstants::*` | Flight constants are centralized here except barometer filter constants, which are local to the sensor task. |
| FSM task construction | `src/app/flight_controller_app.h` | `fsmTask_` member initialization | Passes `FlightState`, `AbortState`, `HighGImuState`, and `TelemetryState` into the FSM. |
| FSM shared state | `src/state/flight_state.h` | `FlightState` | Stores current state and entry timestamps: launch, coast, apogee, drogue, deploy. |
| Barometer telemetry fields | `src/state/telemetry_state.h` | `BarometerTelemetryData` | `rawAltitudeM` is unfiltered; `altitudeM` is filtered and used by FSM. |
| Barometer pressure-to-altitude conversion | `src/sensors/barometer_task.cpp` | `relativeAltitudeM()` | Uses pad reference pressure captured on first valid sample. |
| Barometer filtering | `src/sensors/barometer_task.cpp`, `src/sensors/barometer_task.h` | `BarometerTask::filterAltitude()` and filter member state | 3-sample median followed by EWMA alpha `0.35`. |
| Barometer sample publication | `src/sensors/barometer_task.cpp` | `BarometerTask::tick()` | Updates pressure, reference, raw altitude, filtered altitude, and sample timestamp. |
| Global task periods | `src/app/app_config.cpp`, `src/app/app_config.h` | `DefaultAppConfig::*TaskPeriodMs()` | Barometer period is 50 ms; magnetometer period is 100 ms; FSM period is 10 ms. |
| INIT / abort handling / dispatch | `src/missions/fsm_task.cpp` | `FlightStateMachineTask::tick()` | `INIT -> SAFE`; active abort returns to `SAFE`; state-specific handlers are called here. |
| ARMED launch detection | `src/missions/fsm_task.cpp` | `tickArmed()`, `highGAccelNorm()`, `consumeHighGSample()` | High-g norm `>= 2.0 g` for four fresh high-g samples enters `LAUNCH`. |
| LAUNCH burnout detection | `src/missions/fsm_task.cpp` | `tickLaunch()` | High-g norm `< 1.0 g` for four fresh high-g samples enters `COAST`. |
| COAST apogee detection | `src/missions/fsm_task.cpp` | `tickCoast()`, `consumeBarometerSample()`, `pushApogeeSample()`, `apogeePredictionReady()` | Uses filtered barometer altitude, quality-checked nine-sample fit, `plus2sigma5` aggregation, descent backup, and timeout fallback. |
| Quadratic apogee fit | `src/missions/fsm_task.cpp` | `solveQuadratic()`, `solve3x3()` | Least-squares fit over nine samples using relative seconds from the oldest sample; also computes fit RMSE. |
| Apogee aggregation / quality checks | `src/missions/fsm_task.cpp` | `pushApogeePrediction()`, `plusTwoSigmaApogee()` | Rejects prediction jumps, unstable prediction history, high fit residual, stale barometer gaps, and uses `mean + 2*sigma` over five accepted raw predictions. |
| Forced recovery request | `src/missions/telemetry_task.cpp`, `src/state/flight_state.h` | `handleCommand()`, `forceDeployRequestAllowed()`, `FlightState::forceRecoveryDeployRequested` | Telemetry validates command/auth/state and records a request; it does not directly mutate the FSM state. |
| Forced recovery execution | `src/missions/fsm_task.cpp` | `consumeForceRecoveryDeployRequest()`, `forceRecoveryDeployAllowed()` | FSM consumes the request only in `LAUNCH` or `COAST`, transitions to `APOGEE`, and records execution for ACK. |
| Drogue pyro sequence state | `src/missions/fsm_task.cpp` | `onEnter(State::APOGEE)`, `tickApogee()` | Non-blocking sequence timing is implemented; actual pyro HAL output is TODO. |
| Main deploy decision and sequence | `src/missions/fsm_task.cpp` | `tickDrogue()`, `onEnter(State::DEPLOY)`, `tickDeploy()` | `DROGUE -> DEPLOY` at `<= 200 m AGL` or timeout; actual pyro HAL output is TODO. |
| Telemetry state code mapping | `src/missions/telemetry_task.cpp`, `protocol/include/nura_protocol_v1_lite.h` | `currentFlightStateCode()`, `FlightStateCode` | Downlink status encodes the new state IDs. |
| Protocol documentation state table | `documents/nura_lora_packet_protocol_v1.md` | `State encoding` section | Human-readable packet spec mirrors the code enum. |
| Mock flight data source | `src/hal/mock_flight_data_hal.cpp`, `src/missions/mock_telemetry_source_task.cpp` | `MockFlightDataHAL::read()`, `MockTelemetrySourceTask::tick()` | Generates deterministic nominal, low-apogee, noisy, gust, dropout, and pad-false-accel scenarios; publishes high-g IMU and filtered barometer data for mock builds. |
| Host FSM replay tests | `test/fsm_replay/flight_state_machine_replay.cpp`, `test/fsm_replay/run_fsm_replay_tests.py` | `runReplay()`, `checkFullFlight()` | Compiles the real `fsm_task.cpp` with g++ and verifies launch, coast, apogee, drogue, deploy, ground, abort, forced recovery request, and launch-confirmation behavior without a board. |
| Apogee perturbation experiment | `test/apogee_prediction/evaluate_apogee_prediction.py` | script entrypoint | Replays OpenRocket-derived altitude scenarios with environmental/sensor perturbations and compares apogee aggregation candidates. |

## Verification Commands

Board-free checks:

```bash
python3 test/fsm_replay/run_fsm_replay_tests.py
python3 test/apogee_prediction/evaluate_apogee_prediction.py
pio run -e build
pio run -e mock_test
```

`mock_test` is a bench-only build. It defines `NURA_MOCK_TELEMETRY`,
`NURA_MOCK_AUTO_ARM`, and `NURA_MOCK_SCENARIO_ID=0`, so uploading that
environment runs the nominal deterministic mock flight without real sensors.
Other mock scenarios can be selected by changing `NURA_MOCK_SCENARIO_ID`:

| ID | Scenario |
| ---: | --- |
| 0 | Nominal flight |
| 1 | Low apogee |
| 2 | Barometer noise |
| 3 | Barometer gust / port pulse |
| 4 | Barometer dropout |
| 5 | Pad false acceleration before real launch |
