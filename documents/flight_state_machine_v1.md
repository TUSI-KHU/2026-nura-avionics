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
- Barometer sample rejection and first degraded barometer fault policy.
- High-g / low-g acceleration source fallback for launch and burnout detection.
- Constants that must be represented in code, even if their final values are
  still team-tunable.

Excluded from this version:

- Detailed post-flight log extraction tooling.
- Detailed non-barometer sensor fault policy.
- Hardware-in-the-loop replay with real recorded flight logs.

## Global Operating Assumptions

- After `INIT`, sensor acquisition tasks run continuously in every state.
- The FSM reads the latest shared sensor/state values; it does not directly
  perform sensor bus reads.
- Boot waits for board power and bus settle time before task initialization.
  Individual sensor `begin()` calls retry before the task declares init failure.
- Flight logging records state transitions and decision traces, but the FSM
  itself still owns the recovery decisions.
- Barometer fault fallback is implemented for apogee/main decisions. High-g
  acceleration fallback is implemented only for launch and burnout detection.
  Broader sensor degraded-mode policy is still TODO.
- Launch and burnout detection use a single scalar acceleration norm in g.
  High-g is the primary source; low-g is used only when high-g is unavailable,
  faulted, stale, or non-finite.
- Constants and thresholds must live in code as named constants, not inline
  literals.
- `APOGEE` is a real state in this design. It represents the drogue pyro
  sequence state, not merely an instantaneous event.
- Current avionics firmware auto-transitions from `SAFE` to `ARMED` after
  initialization unless abort is active. The real energetic safety inhibit is
  the external physical pyro/arming hardware path, not a software arming switch
  wired into the avionics MCU in this revision.

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
| `BOARD_POWER_SETTLE_DELAY_MS` | `2000` | ms | Team bench decision; allow the new board sensors, SD, and program flash logger to power up before bus/device init |
| `BUS_SETTLE_DELAY_MS` | `250` | ms | Team bench decision; allow SPI/I2C bus pins and pullups to settle before sensor `begin()` calls |
| `SENSOR_INIT_RETRY_ATTEMPTS` | `5` | attempts | Team bench decision; retry device `begin()` before treating init as failed |
| `SENSOR_INIT_RETRY_DELAY_MS` | `150` | ms | Team bench decision; delay between sensor init retries |
| `ACCEL_FALLBACK_MAX_SAMPLE_AGE_MS` | `50` | ms | Freshness bound for high-g/low-g acceleration samples used by launch and burnout detection |
| `LAUNCH_ACCEL_THRESHOLD_G` | `2.0` | g | Team decision, based on 2025 acceleration-norm logs |
| `LAUNCH_CONFIRM_SAMPLES` | `4` | samples | Team decision; consecutive selected-acceleration samples |
| `BURNOUT_ACCEL_THRESHOLD_G` | `1.0` | g | Team decision; selected acceleration norm below this enters coast |
| `BURNOUT_CONFIRM_SAMPLES` | `4` | samples | Team decision; consecutive selected-acceleration samples |
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
| `MPL3115A2_CONVERSION_TIMEOUT_MS` | `50` | ms | Matches the 50 ms barometer task period; bench trace showed the library default OS128 one-shot path blocking for about 383 ms |
| `MPL3115A2_FAST_BAROMETER_CTRL_REG1` | `0x00` | register value | BAR mode with OS1; team bench decision to keep MPL3115A2 conversion time inside the scheduler budget |
| `BARO_MEDIAN_WINDOW_SAMPLES` | `3` | samples | Spike rejection in barometer task; implementation-local constant |
| `BARO_ALTITUDE_LPF_ALPHA` | `0.35` | ratio | First-order low-pass on median altitude; implementation-local constant, tune after replay |
| `BARO_STALE_FAULT_MS` | `300` | ms | Barometer health policy; stale last-valid sample threshold |
| `BARO_CONSECUTIVE_READ_FAIL_FAULT` | `5` | samples | Barometer health policy; repeated read/transport failures before fault |
| `BARO_BAD_VALUE_TOTAL_FAULT` | `10` | samples | Barometer health policy; total rejected bad values before fault |
| `BARO_BAD_VALUE_CONSECUTIVE_FAULT` | `5` | samples | Barometer health policy; consecutive rejected bad values before fault |
| `BARO_MIN_ALTITUDE_AGL_M` | `-200.0` | m AGL | Loose sanity bound; single out-of-range samples are rejected, not immediate fault |
| `BARO_MAX_ALTITUDE_AGL_M` | `5000.0` | m AGL | Loose sanity bound above expected NURA flight altitude |
| `BARO_STUCK_WINDOW_MS` | `5000` | ms | In-flight stuck detector window, only used after `COAST` |
| `BARO_STUCK_RANGE_M` | `0.2` | m | In-flight stuck detector window range |
| `TILT_MIN_ACCEL_NORM_G` | `0.2` | g | Reject tilt estimates when the low-g acceleration vector is too small to define direction |
| `TILT_MAX_ACCEL_NORM_G` | `3.0` | g | Reject tilt estimates during high-dynamic acceleration |
| `ATTITUDE_ACCEL_CORRECTION_MIN_NORM_G` | `0.7` | g | Enable attitude accel correction only near 1 g |
| `ATTITUDE_ACCEL_CORRECTION_MAX_NORM_G` | `1.3` | g | Disable attitude accel correction during high-dynamic acceleration |
| `ATTITUDE_ACCEL_CORRECTION_GAIN` | `0.8` | 1/s | Proportional correction gain from accel direction to quaternion attitude |
| `ATTITUDE_MAX_DELTA_MS` | `100` | ms | Reject attitude integration across stale IMU sample gaps |
| `BARO_FAULT_ATTITUDE_FALLBACK_MIN_FLIGHT_TIME_MS` | `8000` | ms | Do not allow attitude fallback before this time after launch |
| `BARO_FAULT_ATTITUDE_FALLBACK_TILT_DEG` | `70.0` | deg | Baro-fault late-apogee attitude trigger threshold |
| `BARO_FAULT_ATTITUDE_FALLBACK_CONFIRM_SAMPLES` | `5` | samples | Consecutive fresh low-g attitude samples required |
| `BARO_FAULT_ATTITUDE_FALLBACK_MAX_SAMPLE_AGE_MS` | `150` | ms | Reject stale low-g attitude samples |
| `PYRO_FIRE_DURATION_MS` | `50` | ms | Initial constant, must be verified by ground test |
| `DROGUE_BACKUP_DELAY_MS` | `2000` | ms | Team decision; delay between primary and backup drogue pyro |
| `MAIN_DEPLOY_ALTITUDE_M_AGL` | `200.0` | m | Initial team decision |
| `LANDING_STABLE_WINDOW_SAMPLES` | `20` | samples | Team decision; landing detector window over fresh filtered barometer AGL samples |
| `LANDING_STABLE_ALTITUDE_RANGE_M` | `0.5` | m | Team decision; `max(window) - min(window)` must be below this, not adjacent-sample delta |
| `LANDING_MAX_BARO_SAMPLE_GAP_MS` | `150` | ms | Derived from 50 ms barometer period; reset landing window after stale samples |
| `APOGEE_TIMEOUT_MS` | `12000` | ms | Initial fallback, must be reviewed against simulation / flight data |
| `MAIN_TIMEOUT_MS` | `15000` | ms | Initial fallback, must be reviewed against descent policy |
| `BENCH_FSM_AUTO_ARM_DELAY_MS` | `1000` | ms | Bench integration only; active only with `NURA_BENCH_FSM_AUTOFLOW` |
| `BENCH_FSM_AUTO_LAUNCH_DELAY_MS` | `1000` | ms | Bench integration only; replaces missing arming/launch stimulus |
| `BENCH_FSM_AUTO_BURNOUT_DELAY_MS` | `500` | ms | Bench integration only; lets the existing coast/apogee timers run |
| `BENCH_FSM_AUTO_GROUND_DELAY_MS` | `1000` | ms | Bench integration only; allows GROUND without a real landing barometer profile |

Notes:

- The 4-sample launch and burnout confirmations are intentionally sample-count
  based, not millisecond based.
- The launch and burnout confirmation source is high-g (`H3LIS331DL`) first.
  If high-g is faulted or stale, the low-g IMU acceleration norm is used as a
  degraded fallback. A healthy high-g sample is never mixed with a low-g sample
  for the same detector step.
- Low-g IMU tilt fallback is also used after a barometer fault, but that is a
  separate apogee fallback path.

## State Table

| State | Entry action | Periodic action | Timeout | Fallback | Entry condition from previous state |
| --- | --- | --- | --- | --- | --- |
| `INIT` | Wait for board/bus settle, initialize sensors, initialize calibration, build barometer ground baseline, initialize SD logging and program-flash logging when enabled, force pyro outputs OFF. | Check initialization completion. | TODO init timeout. | `FAULT` or `SAFE`, final policy TBD. | Boot starts here. |
| `SAFE` | Force pyro outputs OFF. Reset flight-local counters and detector state. | Auto-transition to `ARMED` unless abort is active. Sensor tasks continue globally. | None for first implementation. | None. | `INIT` completed. |
| `ARMED` | Reset launch detector consecutive count. Reset launch timestamp. Reset coast/apogee detector scratch state. | Compute selected acceleration norm and update launch confirmation count. | Optional arming timeout TODO. | `SAFE` if abort/disarm policy is later wired into avionics. | `SAFE` completed and abort is not active. |
| `LAUNCH` | Save `launch_time_ms`. Reset burnout confirmation count. TODO: log launch event later. | Continue selected acceleration norm monitoring and update burnout confirmation count. | Optional motor-burn timeout TODO. | `COAST` when motor-burn timeout policy is defined. | In `ARMED`, selected acceleration norm is `>= LAUNCH_ACCEL_THRESHOLD_G` for `LAUNCH_CONFIRM_SAMPLES` consecutive samples. |
| `COAST` | Save `coast_start_ms`. Reset apogee detector scratch state, including max altitude and descent counter. | Push fresh barometer AGL samples into the nine-sample predictor, update predicted-apogee and backup-descent confirmation counts. | `APOGEE_TIMEOUT_MS`. | `APOGEE`. | In `LAUNCH`, selected acceleration norm is `< BURNOUT_ACCEL_THRESHOLD_G` for `BURNOUT_CONFIRM_SAMPLES` consecutive samples. |
| `APOGEE` | Start drogue primary pyro pulse. Save pyro sequence start time. TODO: log drogue sequence start later. | End primary pulse after `PYRO_FIRE_DURATION_MS`; after `DROGUE_BACKUP_DELAY_MS`, fire backup pyro for `PYRO_FIRE_DURATION_MS`; keep all pyro timing non-blocking. | Pyro sequence timeout derived from backup delay plus pulse duration. | `DROGUE`. | Apogee detector asserts apogee, or `COAST` timeout fallback fires. |
| `DROGUE` | Latch drogue sequence complete. Reset main deploy detector scratch state. | Monitor barometer AGL altitude for main deployment threshold. | `MAIN_TIMEOUT_MS`. | `DEPLOY`. | Drogue primary-plus-backup pyro sequence is complete. This does not require proof that the parachute physically opened. |
| `DEPLOY` | Start main pyro pulse. Reset landing detector scratch state. TODO: support backup main pyro only if hardware/team policy requires it. TODO: log main deploy later. | End main pulse after `PYRO_FIRE_DURATION_MS`; after the pulse is complete, push fresh filtered barometer AGL samples into the landing detector. | None for first implementation. | None; timeout policy deferred to sensor fault / recovery policy. | In `DROGUE`, altitude AGL is `<= MAIN_DEPLOY_ALTITUDE_M_AGL`, or `MAIN_TIMEOUT_MS` fallback fires. |
| `GROUND` | Force pyro outputs OFF. TODO: close/flush logs and enter recovery telemetry mode later. | TODO: recovery GPS/telemetry behavior. | None. | None. | Main deploy sequence is complete and the landing detector is stable. |
| `FAULT` | Force pyro outputs OFF. Set fault flag. TODO: log fault later. | Hold fault state. | None. | None. | Initialization failure or future critical fault policy. |

## Bench Integration Auto-Flow

Purpose: support radio/protocol integration tests on the real `main.cpp` app
when the rocket is not actually flying and no arming switch or pyro hardware is
connected yet.

Build gate:

```text
NURA_BENCH_FSM_AUTOFLOW == 1
```

Current PlatformIO usage: this flag is not exposed by the three standard
environments (`main`, `debug`, and `debug_no_lora`). It remains a manual,
bench-only compile flag and must never be added to a flight build.

Inputs and units:

- System time in ms.
- Current FSM state.
- Abort state.

No sensor value is treated as valid just because this mode is enabled. The mode
does not modify barometer, IMU, GPS, LoRa, or storage health.

Allowed states and actions:

```text
SAFE   -> ARMED  after BENCH_FSM_AUTO_ARM_DELAY_MS
ARMED  -> LAUNCH after BENCH_FSM_AUTO_LAUNCH_DELAY_MS
LAUNCH -> COAST  after BENCH_FSM_AUTO_BURNOUT_DELAY_MS
DEPLOY -> GROUND after BENCH_FSM_AUTO_GROUND_DELAY_MS
```

The normal `COAST -> APOGEE`, `APOGEE -> DROGUE`, and `DROGUE -> DEPLOY`
paths still use the existing timer/sequence logic. This keeps the integration
test close to the real state order while removing only the currently missing
bench inputs.

Forbidden states:

- The mode must not run when `abortState.status.active == true`.
- The mode must not be enabled in a flight build.
- The mode must not energize pyro outputs directly. Current pyro behavior is
  still the existing FSM TODO/timing latch because real pyro HAL/pinmap is not
  defined yet.

Threshold sources:

- All four bench delays are team integration-test decisions. They are not
  flight thresholds and must not be used as launch, burnout, or landing logic.

Failure modes considered:

- Missing arming switch: bypassed only in bench build.
- No launch acceleration on the table: bypassed only in bench build.
- No barometer landing profile after main deploy: bypassed only in bench build.
- Active abort: auto-flow is inhibited.

Fallback behavior:

- If auto-flow is disabled, the normal FSM behavior is unchanged.
- If auto-flow reaches `COAST`, existing apogee and main timer fallbacks still
  drive the recovery sequence.

Verification plan:

- Host replay builds both normal FSM and `NURA_BENCH_FSM_AUTOFLOW` variants.
- The bench variant must reach `GROUND` without sensor events and without
  calling the panic handler.
- Hardware integration test, when the bench-only flag is deliberately enabled:
  connect the receiver and verify telemetry reports `SAFE -> ARMED -> LAUNCH
  -> COAST -> APOGEE -> DROGUE -> DEPLOY -> GROUND`.

## Detector Details

### Flight Acceleration Source And Norm

Launch and burnout detection use one selected acceleration source per fresh
sample. The primary source is high-g. The fallback source is low-g.

High-g is usable only when:

```text
highAccelOk == true
AND highG.connected == true
AND highG.hasNewData == true
AND highG.lastUpdatedMs is non-zero and fresh within ACCEL_FALLBACK_MAX_SAMPLE_AGE_MS
AND highG x/y/z values are finite
```

Low-g is usable only when:

```text
lowG.lastUpdatedMs is non-zero and fresh within ACCEL_FALLBACK_MAX_SAMPLE_AGE_MS
AND lowG accel x/y/z values are finite
```

Selection rule:

```text
if high-g is usable:
    use high-g
else if low-g is usable:
    use low-g
else:
    do not update launch/burnout confirmation this tick
```

The detector consumes only one new sample per source timestamp. If high-g is
healthy but has no new timestamp yet, the detector waits for high-g instead of
opportunistically mixing in low-g samples.

For high-g accelerometer values:

```text
accel_norm_g = sqrt(x_g^2 + y_g^2 + z_g^2)
```

For low-g values in m/s^2:

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

If the barometer fault flag is latched, the FSM stops using barometer primary
logic. In `COAST`, it first tries the attitude-based late-apogee fallback and
then falls through to the existing timer fallback. Barometer-fault main deploy
and landing still use timer / no-barometer behavior.

#### Low-g attitude / tilt estimate

`IMUTask` keeps a 6DOF quaternion attitude estimate. Gyro samples are integrated
every valid low-g IMU update. The first valid on-pad acceleration direction is
saved as the pad vertical reference; roll, pitch, yaw, and tilt are all relative
to that boot-time reference frame.

The estimator applies accelerometer correction only when the low-g acceleration
norm is close to 1 g:

```text
ATTITUDE_ACCEL_CORRECTION_MIN_NORM_G
  <= accel_norm_g <=
ATTITUDE_ACCEL_CORRECTION_MAX_NORM_G
```

Outside that window, the estimator uses gyro integration only. This prevents
motor thrust, coast dynamics, and deployment shock from being mistaken for
gravity direction.

The published FSM input is still a scalar tilt angle:

```text
tilt_angle_deg =
    angle_between(initial_pad_vertical, current_estimated_rocket_axis)
```

The roll/pitch/yaw fields are for logging and debugging. Yaw is expected to
drift without a trusted magnetometer correction, so the FSM does not use yaw or
pitch directly. It uses only `tiltAngleDeg`, and only as a degraded late-apogee
confirmation after the barometer has already been faulted.

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
| `barometer.fault` | Latched barometer fault flag | Yes |
| `barometer.faultFlags` | Fault reason bits: read fail, stale, bad value, stuck | Yes |

#### Barometer sample rejection and fault policy

Purpose: keep isolated bad barometer samples out of flight logic without
dropping into degraded mode too aggressively. The policy separates a rejected
sample from a latched sensor fault.

Inputs and units:

- MPL3115A2 pressure in Pa.
- Pad-relative raw altitude in m AGL.
- Filtered altitude in m AGL.
- Sample timestamp in ms.

Allowed states:

- Sample rejection runs in the barometer task regardless of FSM state.
- Stuck detection runs only after `COAST`, currently in `COAST` and `DROGUE`.

Forbidden states:

- Stuck detection must not mark a fault on the pad.
- Stuck detection is not used in `DEPLOY`, because the landing detector also
  looks for a stable altitude window after main deployment.

Rejected sample policy:

```text
if pressure/altitude conversion is non-finite:
    reject this sample only

if raw_altitude_m < BARO_MIN_ALTITUDE_AGL_M:
    reject this sample only

if raw_altitude_m > BARO_MAX_ALTITUDE_AGL_M:
    reject this sample only
```

Rejected samples do not update `lastUpdatedMs`, `rawAltitudeM`, `altitudeM`, or
the altitude filter. They increment bad-value counters. A single `NaN`, `inf`,
or `-4000 m` style sample is not a fault by itself.

Fault policy:

```text
read failures >= BARO_CONSECUTIVE_READ_FAIL_FAULT
OR last valid sample age >= BARO_STALE_FAULT_MS
OR bad values >= BARO_BAD_VALUE_TOTAL_FAULT over the current boot
OR consecutive bad values >= BARO_BAD_VALUE_CONSECUTIVE_FAULT
OR in COAST/DROGUE:
       altitude range over BARO_STUCK_WINDOW_MS <= BARO_STUCK_RANGE_M
```

When any condition is met:

```text
barometer.fault = true
barometer.faultFlags |= reason
barometer.valid = false
```

The fault is intentionally latched for the current boot. Later valid-looking
samples may still be published for telemetry/debugging, but FSM primary
barometer logic ignores them while `barometer.fault == true`.

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

This timer remains active when `barometer.fault == true`.

`APOGEE_TIMEOUT_MS` must not remain unreviewed for flight use and must be
greater than `APOGEE_MIN_FLIGHT_TIME_MS`.

#### Barometer-fault attitude fallback

Allowed only in `COAST` and only when `barometer.fault == true`.

Inputs:

- `imu.tiltValid`
- `imu.tiltAngleDeg`
- `imu.lastUpdatedMs`
- `flightState.launchMs`

Trigger:

```text
barometer.fault == true
AND now_ms - launch_ms >= BARO_FAULT_ATTITUDE_FALLBACK_MIN_FLIGHT_TIME_MS
AND low-g tilt sample is fresh
AND tilt_angle_deg >= BARO_FAULT_ATTITUDE_FALLBACK_TILT_DEG
for BARO_FAULT_ATTITUDE_FALLBACK_CONFIRM_SAMPLES consecutive samples
```

Then:

```text
transition to APOGEE
```

This is deliberately a late confirmation, not a prediction. It exists to avoid
waiting all the way to the timer fallback when the rocket has clearly tipped
over after apogee and the barometer is unavailable.

## Pyro Sequence

All pyro control must be non-blocking. Do not use `delay()` for firing pulses.

## State-Transition Buzzer Feedback

Purpose: make the active FSM transition audible during pad and bench operation.
The buzzer is diagnostic feedback only. It does not arm, inhibit, trigger, or
extend any pyro action, and a buzzer failure must not change the flight state.

Input and output:

- Input: the previous state, next state, and transition timestamp supplied by
  `FlightStateMachineTask::transitionTo()`.
- Output: D2 through `BuzzerHAL`, using non-blocking `tone()`/`noTone()` calls.
- Allowed operation: state transition feedback only.
- Forbidden operation: raw sensor input, LoRa packet parsing, pyro requests,
  arming authorization, and state-transition decisions.

Pattern assignment:

| Transition | Pattern | Duration |
| --- | --- | --- |
| `INIT -> SAFE` | Seven medium-pitch `2200 Hz` tones | `90 ms` ON / `80 ms` gap, seven times |
| `SAFE -> ARMED` | Continuous, full-amplitude `3000 Hz` alert tone | 5 s |
| All other state transitions | Five fast `2400 Hz` tones | `80 ms` ON / `80 ms` gap, five times |

Threshold source:

- The seven-beep init indication, 5 s flat arming alert, and five-beep
  transition convention are explicit team decisions.
- D2 is already driven with the maximum 3.3 V `tone()` amplitude. `3000 Hz` is
  the initial irritating alert frequency; the `90/80 ms` init cadence and
  `80/80 ms` transition cadence are initial audible defaults. They are not
  flight-control thresholds and may be retuned after a bench audibility test.

Failure modes and fallback:

- `BuzzerHAL::begin()` failure logs a warning and leaves the FSM muted; it does
  not cause `FAULT` or block deployment logic.
- The `INIT -> SAFE` seven-tone sequence queues the `SAFE -> ARMED` flat alert
  so the init indication finishes first. A later transition replaces the
  current diagnostic pattern immediately.
- Every pattern is serviced from the normal FSM tick without `delay()`, so it
  cannot intentionally stall sensor acquisition, timeout processing, or pyro
  pulse timing.

Verification plan:

- Host replay: `checkBuzzerStateTransitionPatterns()` verifies the seven-tone
  init sequence, queued flat alert, and five-tone transition pattern.
- Bench: with no e-matches connected, flash `debug_no_lora`, confirm D2 audio,
  and optionally scope the frequency/gaps while the board is shaken through
  `ARMED -> LAUNCH -> COAST`. Pyro remains dry-run unless the separate
  `NURA_ENABLE_PYRO_OUTPUTS` bench gate is deliberately enabled.

Physical channel mapping:

```text
Pyro 1 / Drogue -> BoardPinMap::DroguePyro
Pyro 2 / Main   -> BoardPinMap::MainPyro
```

The MOSFET HAL assumes active-high TC4452/MOSFET-driver inputs. It drives both
GPIO inputs listed for a channel together and treats the `sense` pin as input
only. Continuity interpretation is still deferred until the sense circuit is
characterized.

Allowed states and inputs:

- Drogue/Pyro 1 ON is allowed only on `APOGEE` entry and the retry point inside
  `APOGEE`.
- Main/Pyro 2 ON is allowed only on `DEPLOY` entry.
- `SAFE`, `GROUND`, and `FAULT` force all pyro outputs OFF.
- The only timing inputs used by this logic are `apogeeMs`, `deployMs`,
  `PYRO_FIRE_DURATION_MS`, and `DROGUE_BACKUP_DELAY_MS`, all in milliseconds.
- The pyro HAL does not read sensors, parse telemetry packets, or decide state
  transitions. It only executes requests from the FSM.

Failure modes considered:

- Boot/init failure: if physical pyro outputs are enabled and the HAL detects
  any overlap between D21 power sense and a pyro output, FSM init fails and the
  app enters the existing panic path. The final pinmap D28/D29/D36/D37 has no
  such overlap.
- Software reset or disarm: entering `SAFE`, `GROUND`, or `FAULT` calls
  `allOff()`.
- Continuity/sense failure: not acted on yet because the sense circuit has not
  been characterized. This remains a TODO before flight qualification.

Verification plan:

- Host replay: `checkPyroOutputSequence()` verifies Drogue ON/OFF/retry and
  Main ON/OFF call order without hardware.
- Bench GPIO test: with no e-matches attached, define `NURA_ENABLE_PYRO_OUTPUTS`
  and scope/LED-test D28/D29/D36/D37.
- Hardware integration: verify continuity/sense readings before allowing arming
  gates to depend on them.

Build-time safety gate:

```text
default build:
    FSM calls the HAL, but MosfetPyroHAL is dry-run and does not drive GPIOs

NURA_ENABLE_PYRO_OUTPUTS:
    MosfetPyroHAL configures and drives the GPIO pins

NURA_ENABLE_PYRO_OUTPUTS without NURA_ALLOW_PYRO_POWER_SENSE_PIN_CONFLICT:
    init fails if a future pinmap maps battery voltage sense and a pyro output
    to the same pin
```

`NURA_ALLOW_PYRO_POWER_SENSE_PIN_CONFLICT` remains a bench-only escape hatch
for an intentionally modified pinmap. It is not needed for the final D21,
D28/D29/D36/D37 assignment and must not be used in a flight build.

Drogue sequence in `APOGEE`:

```text
t = apogee_entry_ms

Drogue/Pyro 1 ON at t
Drogue/Pyro 1 OFF at t + PYRO_FIRE_DURATION_MS

Drogue/Pyro 1 retry ON at t + DROGUE_BACKUP_DELAY_MS
Drogue/Pyro 1 retry OFF at t + DROGUE_BACKUP_DELAY_MS + PYRO_FIRE_DURATION_MS

after retry OFF:
    transition to DROGUE
```

Main sequence in `DEPLOY`:

```text
t = deploy_entry_ms

main ON at t
main OFF at t + PYRO_FIRE_DURATION_MS

after main OFF:
    mark main sequence complete
    start accepting landing-detector barometer samples
```

Landing detector in `DEPLOY`:

```text
allowed only after mainSequenceComplete == true
input: filtered barometer altitude AGL, fresh sample timestamp

for each fresh barometer sample:
    if sample gap > LANDING_MAX_BARO_SAMPLE_GAP_MS:
        reset landing window
    push altitude into a 20-sample ring buffer

if the ring buffer is full
AND max(altitude_window) - min(altitude_window) <= LANDING_STABLE_ALTITUDE_RANGE_M:
    transition to GROUND

```

The detector intentionally uses the full 20-sample window range. It must not be
implemented as "20 adjacent differences below 0.5 m", because a slow parachute
descent can have adjacent 50 ms deltas below 0.5 m while still airborne.
A `DEPLOY -> GROUND` timeout fallback is intentionally deferred. A fixed timeout
can become an early-ground false positive if `DROGUE -> DEPLOY` entered by
timeout while the rocket was still high.

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

- Program-flash/SD logging policy and storage buffer sizing.
- Broader non-barometer degraded-mode transitions beyond high-g/low-g launch
  and burnout fallback.
- Barometer filter tuning.
- Tune landing detector thresholds with actual main-parachute descent logs.
- Decide whether a guarded `DEPLOY -> GROUND` timeout fallback is needed.
- Pyro continuity and battery voltage gating.
- Flight-log replay verification tests.

## Implementation Map

This table maps the documented logic to the current code so later reviews can
check implementation drift quickly.

| Logic / data | File | Code location / function | Notes |
| --- | --- | --- | --- |
| State enum and printable names | `src/core/states.h` | `enum class State`, `stateName()` | IDs match the telemetry state code table. |
| Mission thresholds and timeouts | `include/nura_constants.h` | `NuraConstants::Flight::*` | Flight constants are centralized here; protocol layout and board pins stay in their own source-of-truth files. |
| FSM task construction | `src/app/flight_controller_app.h` | `fsmTask_` member initialization | Passes `FlightState`, `AbortState`, `HighGImuState`, `ImuState`, and `TelemetryState` into the FSM. |
| FSM shared state | `src/state/flight_state.h` | `FlightState` | Stores current state and entry timestamps: launch, coast, apogee, drogue, deploy. |
| Low-g quaternion attitude estimate | `src/sensors/imu_task.cpp`, `src/state/imu_state.h` | `updateAttitudeEstimate()`, `ImuData::rollDeg/pitchDeg/yawDeg/tiltAngleDeg` | Integrates gyro, corrects with accel direction only near 1 g, and publishes roll/pitch/yaw plus the scalar FSM tilt. |
| Barometer telemetry fields | `src/state/telemetry_state.h` | `BarometerTelemetryData` | `rawAltitudeM` is unfiltered; `altitudeM` is filtered and used by FSM. |
| Barometer pressure-to-altitude conversion | `src/sensors/barometer_task.cpp` | `relativeAltitudeM()` | Uses pad reference pressure captured on first valid sample. |
| MPL3115A2 conversion timing | `src/hal/mpl3115a2_hal.cpp`, `include/nura_constants.h` | `MPL3115A2HAL::begin()`, `MPL3115A2HAL::poll()`, `kFastBarometerCtrlReg1`, `kConversionTimeoutMs` | Overrides the Adafruit library default OS128 one-shot setup with BAR/OS1 and non-blocking one-shot polling. `PENDING` yields the scheduler without counting as a read failure; conversion timeout is counted as a read failure. |
| Barometer filtering | `src/sensors/barometer_task.cpp`, `src/sensors/barometer_task.h` | `BarometerTask::filterAltitude()` and filter member state | 3-sample median followed by EWMA alpha `0.35`. |
| Barometer sample publication | `src/sensors/barometer_task.cpp` | `BarometerTask::tick()` | Updates pressure, reference, raw altitude, filtered altitude, and sample timestamp. |
| Barometer sample rejection / fault latch | `src/sensors/barometer_task.cpp`, `src/state/telemetry_state.h` | `recordReadFailure()`, `recordBadValue()`, `markFault()`, `BarometerTelemetryData` | Rejects isolated bad samples; latches read-fail, stale, bad-value, and stuck fault flags after the configured thresholds. |
| Global task periods | `src/app/app_config.cpp`, `src/app/app_config.h` | `DefaultAppConfig::*TaskPeriodMs()` | Barometer period is 50 ms; magnetometer period is 100 ms; FSM period is 10 ms. |
| Watchdog recovery scheduling | `src/missions/watchdog_task.cpp`, `src/sensors/imu_task.cpp`, `src/sensors/high_g_imu_task.cpp`, `src/sensors/magnetometer_task.cpp`, `src/sensors/barometer_task.cpp` | `WatchdogTask::tick()`, sensor `recover()` / `initialize()` paths | Runtime recovery attempts are bounded to one immediate sensor `begin()` attempt per watchdog recovery interval. The recovery path avoids multi-attempt `delay()` loops that can stall FSM, telemetry, and sensor scheduling. |
| Nonblocking LoRa TX | `src/hal/sx1262_lora_hal.cpp`, `src/missions/telemetry_task.cpp` | `Sx1262LoRaHAL::send()`, `Sx1262LoRaHAL::service()`, `TelemetryTask::tick()` | TX starts with `startTransmit()` and completes from periodic service on DIO1 so LoRa airtime does not block FSM/sensor scheduling. ACKs are retained until TX starts; FAST/GPS samples may be skipped until the next period if TX cannot start. |
| INIT / abort handling / dispatch | `src/missions/fsm_task.cpp` | `FlightStateMachineTask::tick()` | `INIT -> SAFE`; active abort returns to `SAFE`; state-specific handlers are called here. |
| ARMED launch detection | `src/missions/fsm_task.cpp` | `tickArmed()`, `consumeFlightAccelSample()` | Selected acceleration norm `>= 2.0 g` for four fresh samples enters `LAUNCH`; high-g is primary, low-g is fallback. |
| LAUNCH burnout detection | `src/missions/fsm_task.cpp` | `tickLaunch()`, `consumeFlightAccelSample()` | Selected acceleration norm `< 1.0 g` for four fresh samples enters `COAST`; high-g is primary, low-g is fallback. |
| COAST apogee detection | `src/missions/fsm_task.cpp` | `tickCoast()`, `consumeBarometerSample()`, `pushApogeeSample()`, `apogeePredictionReady()` | Uses filtered barometer altitude, quality-checked nine-sample fit, `plus2sigma5` aggregation, descent backup, and timeout fallback. |
| Barometer-fault attitude fallback | `src/missions/fsm_task.cpp` | `baroFaultAttitudeFallbackReady()` | When barometer is faulted in `COAST`, requires launch+8 s, fresh valid tilt, tilt `>= 70 deg`, and five consecutive samples before `APOGEE`; timer fallback remains active. |
| In-flight barometer stuck detection | `src/missions/fsm_task.cpp` | `trackBarometerStuck()`, `markBarometerFault()` | Tracks 5 s altitude range only in `COAST`/`DROGUE`; latches `BARO_FAULT_STUCK` and leaves recovery to timer fallback. |
| Quadratic apogee fit | `src/missions/fsm_task.cpp` | `solveQuadratic()`, `solve3x3()` | Least-squares fit over nine samples using relative seconds from the oldest sample; also computes fit RMSE. |
| Apogee aggregation / quality checks | `src/missions/fsm_task.cpp` | `pushApogeePrediction()`, `plusTwoSigmaApogee()` | Rejects prediction jumps, unstable prediction history, high fit residual, stale barometer gaps, and uses `mean + 2*sigma` over five accepted raw predictions. |
| Forced recovery request | `src/missions/telemetry_task.cpp`, `src/state/flight_state.h` | `handleCommand()`, `forceDeployRequestAllowed()`, `FlightState::forceRecoveryDeployRequested` | Telemetry validates command/auth/state and records a request; it does not directly mutate the FSM state. |
| Forced recovery execution | `src/missions/fsm_task.cpp` | `consumeForceRecoveryDeployRequest()`, `forceRecoveryDeployAllowed()` | FSM consumes the request only in `LAUNCH` or `COAST`, transitions to `APOGEE`, and records execution for ACK. |
| Pyro HAL | `src/hal/pyro_output.h`, `src/hal/mosfet_pyro_hal.h`, `src/hal/mosfet_pyro_hal.cpp` | `IPyroOutput`, `MosfetPyroHAL` | MOSFET output abstraction; default dry-run, physical GPIO only with `NURA_ENABLE_PYRO_OUTPUTS`; blocks any future power-sense/pyro pin overlap. |
| Drogue pyro sequence state | `src/missions/fsm_task.cpp` | `onEnter(State::APOGEE)`, `tickApogee()` | Non-blocking Drogue/Pyro 1 pulse, then same-channel retry pulse after `DROGUE_BACKUP_DELAY_MS`; transitions to `DROGUE` after retry off. |
| Main deploy decision and sequence | `src/missions/fsm_task.cpp` | `tickDrogue()`, `onEnter(State::DEPLOY)`, `tickDeploy()` | `DROGUE -> DEPLOY` at `<= 200 m AGL` or timeout; non-blocking Main/Pyro 2 pulse, then marks main sequence complete. |
| Landing / ground transition | `src/missions/fsm_task.cpp` | `tickDeploy()`, `consumeLandingSample()`, `landingStable()` | After main pulse completion, uses 20 fresh filtered barometer AGL samples and transitions to `GROUND` when window range is `<= 0.5 m`; timeout fallback is deferred. |
| Telemetry state code mapping | `src/missions/telemetry_task.cpp`, `protocol/include/nura_protocol_v1_lite.h` | `currentFlightStateCode()`, `FlightStateCode` | Downlink status encodes the new state IDs. |
| Protocol documentation state table | `documents/nura_lora_packet_protocol_v1.md` | `State encoding` section | Human-readable packet spec mirrors the code enum. |
| Mock flight data source | `src/hal/mock_flight_data_hal.cpp`, `src/missions/mock_telemetry_source_task.cpp` | `MockFlightDataHAL::read()`, `MockTelemetrySourceTask::tick()` | Generates deterministic nominal, low-apogee, noisy, gust, dropout, and pad-false-accel scenarios; publishes high-g IMU and filtered barometer data for mock builds. |
| Host FSM replay tests | `test/fsm_replay/flight_state_machine_replay.cpp`, `test/fsm_replay/run_fsm_replay_tests.py` | `runReplay()`, `checkFullFlight()`, `checkPyroOutputSequence()` | Compiles the real `fsm_task.cpp` with g++ and verifies launch, coast, apogee, drogue, deploy, ground, abort, forced recovery request, launch-confirmation behavior, and pyro output call order without a board. |
| Apogee perturbation experiment | `test/apogee_prediction/evaluate_apogee_prediction.py` | script entrypoint | Replays OpenRocket-derived altitude scenarios with environmental/sensor perturbations and compares apogee aggregation candidates. |

## Verification Commands

Board-free checks:

```bash
python3 test/fsm_replay/run_fsm_replay_tests.py
python3 test/apogee_prediction/evaluate_apogee_prediction.py
pio run -e main
pio run -e debug
pio run -e debug_no_lora
```
