# NURA Flight Logging V1

Status: Draft implementation
Target storage: Teensy 4.1 U3 program flash primary, microSD mirror
Target controller: Teensy 4.1

## Purpose

Flight logging must preserve enough data to reconstruct what the vehicle and
flight computer saw, and why the state machine made each recovery decision. The
logger must not block flight logic, and loss of the removable SD card must not
destroy the primary flight record.

## Storage Policy

### Primary Program Flash

The current avionics board uses the Teensy 4.1 U3 program flash
(`W25Q64JVXGIM` on the PJRC schematic) as the non-removable log target. This is
the same physical flash family that stores firmware, so the backend mounts only
a fixed tail region of program flash through Teensy `LittleFS_Program`.

Initial allocation: `6 MB`, defined by
`NuraConstants::Logger::kFlightLogProgramFlashBytes`. The remaining flash stays
available for firmware. If the firmware image ever grows too large for this
allocation, `LittleFS_Program::begin()` fails and the SD mirror can still carry
the log.

The enabled backend is file-backed:

```text
on boot:
    mount the configured program-flash tail region with LittleFS_Program
    create/open /NURA_LOG/FLxxx.NLG

while logging:
    append complete encoded frames
    rotate to a new file segment after 256 KB
    delete the lowest-numbered older log file if free space is low

on GROUND:
    write the final GROUND event
    flush pending records
    stop all further flash writes
```

The stop-on-ground rule prevents long recovery waits from overwriting the actual
boost, coast, apogee, drogue, and main-deploy data.

Important operational note: uploading firmware may change or erase the
program-flash filesystem region depending on the loader and image layout. Treat
U3 as the flight blackbox during a powered mission, and treat microSD as the
easier post-test extraction path.

### microSD Mirror

microSD receives the same encoded binary records as the storage interface sees.
The SD backend writes one `.NLG` file per boot under `/NURA_LOG/` and selects
the first free name from `FL000.NLG` through `FL999.NLG`.

SD failure must not affect the flight state machine. When flash and SD are both
enabled, `FlightLogMirrorStorage` keeps writing as long as at least one target
accepts the record. A failed target is disabled for the rest of the session.

The current three PlatformIO environments define
`NURA_DISABLE_PROGRAM_FLASH_LOG`: program flash is represented by a
`NullFlightLogStorage`, while SD remains the required active logger. Thus every
current `main`, `debug`, and `debug_no_lora` build records the complete log
stream to SD without mounting or writing program flash. Remove that flag only
after program-flash hardware validation is complete.

## Buffering

Two buffers are used:

| Buffer | Size | Purpose |
| --- | ---: | --- |
| Encoded RAM FIFO | 16 KB | Holds complete encoded records before storage writes. If full, oldest records are dropped first. |
| Program flash filesystem | 6 MB | LittleFS region at the tail of U3 program flash. The backend appends complete encoded frames to `.NLG` files. |

The RAM FIFO stores already encoded binary frames, not C++ structs. This keeps
the storage backend independent from sensor/state structures.

At 50 Hz:

```text
80 B record  * 50 Hz ~= 4.0 KB/s
128 B record * 50 Hz ~= 6.4 KB/s
```

A 16 KB RAM FIFO gives roughly 2.5 to 4 seconds of shock absorption against
short write stalls. A 4 KB-only RAM queue would be too small once SD mirroring
or flash erase latency is included.

## Record Classes

All records share one binary frame:

```text
frame_header:
    magic
    version
    type
    payload_length
    sequence
    timestamp_ms
payload:
    type-specific fixed-size binary data
crc16:
    CRC over header + payload
```

The CRC lets post-flight tools discard a final torn record after brownout or
impact.

### FAST Sample

Default target period: 20 ms to match the existing logger task rate and capture
high-rate IMU behavior.

Contents:

- timestamp and current FSM state.
- low-g IMU acceleration and gyro.
- high-g acceleration and high-g raw counts.
- barometer pressure, raw altitude, and filtered altitude.
- tilt angle.
- battery voltage.
- health flags.
- latest FSM decision sequence number.

### SLOW Sample

Default target period: 100 ms.

Contents:

- magnetometer raw counts and uT values.
- GPS latitude, longitude, altitude, speed, course, HDOP, satellites.
- GPS parser counters.
- barometer fault counters and flags.
- storage and recovery health flags.

### EVENT Record

Written when discrete events occur:

- BOOT / logger init.
- state transition.
- forced recovery request accepted/rejected/executed.
- pyro sequence timing events once real pyro HAL exists.
- GROUND final stop.

### DECISION Record

Written whenever the FSM records a decision trace. The FSM pushes decisions
into a fixed 16-entry queue in `FlightState`, and `FlightLogTask` drains that
queue into DECISION frames so multiple decisions made inside one scheduler tick
are not silently overwritten.

Examples:

- launch acceleration threshold observation / acceptance.
- burnout threshold observation / acceptance.
- apogee quadratic prediction accepted or rejected, including fit RMSE,
  predicted apogee, margin, and confirmation count.
- apogee descent backup observation.
- barometer-fault tilt fallback observation / acceptance.
- apogee timer fallback.
- main altitude / main timer decision.
- landing stability decision.

This is the record family that answers, "why did the FSM fire recovery here?"

## Raw And Filtered Data

Store both raw and processed values when they exist:

| Sensor | Raw | Processed |
| --- | --- | --- |
| low-g IMU | TODO: raw register counts if HAL exposes them later | acceleration m/s^2, gyro dps, R/P/Y, tilt |
| high-g accelerometer | raw X/Y/Z counts | acceleration g and m/s^2 |
| barometer | pressure Pa, raw AGL altitude | filtered AGL altitude, fault counters |
| magnetometer | raw X/Y/Z counts | uT X/Y/Z |
| GPS | parser counters | fix, lat/lon, altitude, speed, course |
| LoRa | TODO later | RSSI/SNR/status if needed |

The first implementation stores every currently exposed raw field. Low-g raw
register counts are marked TODO because the current low-g HAL exposes SI units
only.

## Failure Policy

- The microSD mirror is mandatory in the current avionics build. If the SD card
  is absent, cannot be mounted, cannot create `/NURA_LOG`, or cannot open a new
  `.NLG` file, `FlightLogTask::init()` fails and the application enters the
  init-failure panic path.
- SD mount and `/NURA_LOG` directory creation are retried during init
  (`20 x 250 ms`) after the board-level power settle delay. This keeps SD
  mandatory while tolerating card/power settle timing on the PCB.
- The application attempts to mount/open the mandatory log storage before sensor
  bus and task initialization, then `FlightLogTask::init()` reuses the
  already-open backend. This avoids observed SDIO mount failures after SPI
  sensor bring-up on the current PCB/bench stack.
- `src/main.cpp` performs a boot preflight `SD.sdfs.begin(SdioConfig(FIFO_SDIO))`
  before constructing `FlightControllerApp`. On the current Teensy 4.1 PCB
  stack, claiming the built-in SDIO bus before the full app object is
  constructed prevents `SD_CARD_ERROR_CMD0` failures during the later logger
  init. `FlightLogTask::init()` still owns the init-fatal storage decision and
  file creation.
- On Teensy built-in SD, the backend mounts `SD.sdfs` directly with
  `SdioConfig(FIFO_SDIO)` instead of `SD.begin(BUILTIN_SDCARD)`. The Teensy
  compatibility wrapper drives the DAT3 card-detect line pulldown after a
  failed early probe; direct SdFat mounting avoids poisoning later boot retries.
- Program flash may be used as the primary logging backend, but it does not make
  an absent SD card acceptable in this revision.
- If the program-flash backend fails but SD still writes, continue logging and
  report degraded storage health as needed.
- If every active persistent backend fails after init, set
  `TelemetryState.health.storageOk = false`.
- If RAM FIFO is full, drop oldest encoded records first.
- Stop writing once `GROUND` is reached and pending records are flushed.
- Keep the legacy serial event logger separate; it is for bench visibility, not
  the primary flight record.

## Timestamp Policy

- SD file timestamps use a Teensy SdFat date/time callback set from the firmware
  build date and time (`__DATE__`, `__TIME__`).
- This is a build/upload-time approximation. The board currently has no trusted
  real-time clock or GNSS time handoff during early init, so true launch time is
  still captured inside log records by monotonic milliseconds rather than FAT
  directory metadata.

## Implementation Map

| Item | File |
| --- | --- |
| Binary record definitions and encoder | `src/logging/flight_log_record.*` |
| 16 KB encoded RAM FIFO | `src/logging/flight_log_ram_buffer.*` |
| Storage backend interface / null backend | `src/logging/flight_log_storage.h` |
| Mirror fan-out backend | `src/logging/flight_log_mirror_storage.*` |
| U3 program flash `.NLG` backend | `src/logging/program_flash_flight_log_storage.*` |
| microSD `.NLG` backend | `src/logging/sd_flight_log_storage.*` |
| Periodic snapshot/event/decision task | `src/missions/flight_log_task.*` |
| FSM decision trace source | `src/state/flight_state.h`, `src/missions/fsm_task.cpp` |
| Dump decoder / CSV exporter | `tools/decode_flight_log.py` |
| Host-side logger tests | `test/logging/*` |

## Known TODOs

- Decide whether post-flight tooling should merge multiple 256 KB `.NLG`
  segments automatically.
- Measure flash erase latency and SD write latency on the actual board.
- Add low-g raw register counts if the HAL exposes them.
