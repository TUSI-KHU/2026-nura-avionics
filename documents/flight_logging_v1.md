# NURA Flight Logging V1

Status: Implemented; hardware latency qualification pending

Target controller: Teensy 4.1

Persistent storage:

- Primary: Teensy 4.1 U3 W25Q128 QSPI NOR flash, 16 MiB raw journal
- Mirror: built-in SDIO microSD, one 16 MiB preallocated file per boot

## 1. Purpose and safety boundary

The flight logger records the sensor values seen by the flight computer and the
evidence used by the recovery state machine. Logging must never delay sensor
sampling, FSM execution, telemetry, or pyro sequencing for a flash erase or SD
card busy interval.

The scheduler is cooperative. Therefore a storage function which waits for a
medium to finish is equivalent to stopping every task. The runtime storage
contract is consequently:

1. `append()` only copies an already encoded frame into fixed RAM.
2. `service()` performs at most one bounded controller transaction or starts one
   medium operation and returns.
   `FlightLogTask` invokes it at most once per scheduler tick, including while
   completing the GROUND flush.
3. QSPI page-program and erase completion are checked with one status-register
   read on a later scheduler tick. There is no WIP busy-wait in runtime code.
4. SD data is submitted only when `FsFile::isBusy()` is false. One call submits
   one 512-byte SDIO FIFO block.
5. No runtime path calls LittleFS, FAT allocation, FAT sync, file truncate, or
   file close.

Blocking operations are permitted only before `Scheduler::init()`:

- SD mount, directory creation, file creation and preallocation
- QSPI JEDEC identification and journal scan
- bounded completion wait for a NOR operation left active across an MCU-only reset
- first QSPI sector erase and journal-header program when no journal exists

Those operations can delay boot, but cannot delay a running flight task.

## 2. Logical record stream

`FlightLogTask` creates one logical byte stream. The stream contains complete
application frames and is identical before the SD and QSPI physical wrappers
are applied.

```text
FrameHeader (14 bytes)
    magic             uint16  0x4E4C, "NL"
    version           uint8   1
    type              uint8
    payload_length    uint16
    sequence          uint32
    timestamp_ms      uint32

payload               payload_length bytes
crc16_ccitt           uint16, header + payload
```

Chunk and sector boundaries are not application-frame boundaries. A frame can
cross a QSPI page or SD block. Recovery tooling first reconstructs the logical
stream and then validates each application frame CRC.

## 3. Production rate and RAM budget

Current encoded sizes are:

| Record | Encoded size | Rate | Byte rate |
| --- | ---: | ---: | ---: |
| FAST | 82 B | 50 Hz | 4,100 B/s |
| SLOW | 76 B | 10 Hz | 760 B/s |
| EVENT/DECISION | variable | event driven | normally small |

Nominal periodic production is approximately 4.86 kB/s.

| Buffer | Size | Ownership | Purpose |
| --- | ---: | --- | --- |
| Encoded frame FIFO | 16 KiB | `FlightLogTask` | Absorbs aggregate storage backpressure; drops oldest complete frames on overflow. |
| QSPI byte queue | 8 KiB | QSPI backend | Two 4 KiB-equivalent staging banks; drains as 240-byte journal payloads. |
| SD byte queue | 8 KiB | SD backend | Two 4 KiB-equivalent staging banks; drains as 488-byte journal payloads. |

The storage queues provide about 1.6 seconds of local buffering at the nominal
rate. The upstream 16 KiB FIFO adds approximately 3.3 seconds. Records are moved
from the upstream FIFO only when every currently active backend has room, so a
short busy period does not cause the two copies to diverge.

## 4. QSPI raw journal

LittleFS is not used by the flight logger. The previous implementation called
`LittleFS_QSPIFlash::write()` and `usedSize()` from the cooperative task. The
Teensy LittleFS W25Q128 profile can wait up to 3 ms for page program and up to
2 seconds for its selected 64 KiB erase. That behavior is incompatible with the
flight scheduler.

The new backend uses the W25Q128 commands directly:

| Operation | Unit | Runtime behavior |
| --- | ---: | --- |
| Read status `0x05` | 1 byte | One bounded FlexSPI2 IP transaction |
| Page program `0x02` | 256 B | Command/data submitted, WIP polled on later ticks |
| Sector erase `0x20` | 4 KiB | Command submitted, WIP polled on later ticks |
| Read `0x03` | up to 256 B | Used for header scan and post-program verification |

The FlexSPI2 IP transaction itself has a 1 ms timeout. This is a controller
command bound, not permission to wait for NOR program or erase completion.

### 4.1 Sector layout

W25Q128 is divided into 4096 circular 4 KiB sectors. Each sector contains one
header page and fifteen data pages.

```text
4 KiB sector
    page 0: SectorHeader + 0xFF padding
    page 1..15: PageHeader + up to 240 stream bytes + 0xFF padding
```

`SectorHeader`, 16 bytes, packed little-endian:

```text
magic                 uint32  0x4E534543, "NSEC"
sector_sequence       uint32
first_stream_offset   uint32
version               uint16  1
crc16                 uint16  CRC of header with this field zero
```

`PageHeader`, 16 bytes, packed little-endian:

```text
magic                 uint32  0x4E504147, "NPAG"
stream_offset         uint32
payload_length        uint16  1..240
payload_crc16         uint16
header_crc16          uint16  CRC of header with this field zero
version               uint8   1
flags                 uint8   reserved
```

The page is consumed from RAM only after WIP clears and a complete 256-byte
read-back equals the submitted page. A torn page is rejected by its header or
payload CRC at the next boot.

### 4.2 Circular retention and erase-ahead

Sector sequence numbers order the circular journal. At boot, only 4096 sector
headers and the newest sector's data pages are scanned. The next valid page and
logical stream offset are then recovered.

Two erased sectors are maintained ahead of the active write sector. Erase is an
asynchronous state:

```text
IDLE -> issue 4 KiB erase -> ERASE_BUSY
ERASE_BUSY -> one status poll per logger tick
ready -> verify erased header bytes -> IDLE
```

At 4.86 kB/s, one sector carries 3600 logical bytes, or about 0.74 seconds of
data. Two erased sectors therefore provide about 1.48 seconds of write reserve.
The 8 KiB backend queue absorbs a worst-case 4 KiB erase interval while sensor
and FSM tasks continue running.

When the physical ring wraps, the next oldest sector is erased and reused. Once
GROUND flushing is requested, speculative erase-ahead stops. Only queued data
is programmed, preserving the final flight record.

Effective QSPI logical capacity is approximately 14.75 MB after headers and
padding, or about 50 minutes at the current periodic rate.

## 5. microSD journal file

The SD backend mounts the Teensy built-in SDIO interface with
`SdioConfig(FIFO_SDIO)`. DMA mode is intentionally not used: the bundled SdFat
DMA path waits for DMA completion inside the call, while FIFO mode permits one
sector to be loaded into the SDHC FIFO and completed by the controller/card in
the background.

At init, `/NURA_LOG/FLxxx.NLG` is created and preallocated to 16 MiB. This avoids
cluster allocation during flight. The allocation metadata is synchronized
before the scheduler starts.

Preallocated FAT clusters can contain old bytes, and the file cannot be safely
truncated or closed without a potentially blocking metadata sync. Therefore the
file is a sequence of self-validating 512-byte blocks rather than a naked frame
stream.

`BlockHeader`, 24 bytes, packed little-endian:

```text
magic                 uint32  0x4E534442, "NSDB"
session_id            uint32
block_sequence        uint32
stream_offset         uint32
payload_length        uint16  1..488
payload_crc16         uint16
header_crc16          uint16  CRC of header with this field zero
version               uint8   1
flags                 uint8   reserved
```

The remaining 488 bytes contain the logical stream and `0xFF` padding. The
session ID is derived from build time, file index and boot-time microseconds.
The decoder accepts only contiguous block sequence and stream offsets belonging
to the session in block zero. Stale preallocated cluster contents are ignored.

Runtime write sequence:

```text
if file.isBusy():
    return immediately
if at least 488 queued bytes, or final GROUND flush has a partial block:
    build one 512-byte CRC block
    file.write(block, 512)
    return
```

The 16 MiB boundary is enforced. The backend faults instead of extending the
file and causing an in-flight FAT allocation. It carries approximately 15.98 MB
of logical payload, or roughly 55 minutes at the current periodic rate.

An SD busy interval longer than 1200 ms marks only the SD backend failed. The
value is below the approximately 1.6-second SD queue capacity at nominal rate;
it must be checked against the exact flight SD card during qualification.

## 6. Mirror and failure policy

Both QSPI and SD are required during initialization in the current flight build.
After initialization they fail independently:

- one backend failure disables only that backend;
- the surviving backend continues accepting the logical stream;
- `storageOk` becomes false only when every persistent backend is offline;
- storage failure never changes the flight state or recovery decision;
- transient backpressure pauses upstream draining instead of immediately
  creating different streams on SD and QSPI;
- if all RAM buffering is exhausted, the 16 KiB producer FIFO drops the oldest
  complete records and increments its drop counter.

QSPI faults on controller-command timeout, media-operation timeout, erase
verification failure, or page read-back mismatch. SD faults on write failure,
preallocated boundary exhaustion, or the bounded busy timeout.

## 7. GROUND finalization

GROUND finalization is itself asynchronous:

1. enqueue `GROUND_STOP` once;
2. stop generating FAST/SLOW records;
3. move a bounded number of remaining frames to backend queues each tick;
4. request backend flush when the producer FIFO is empty;
5. QSPI writes one final partial page and SD writes one final padded block;
6. wait through normal `service()` polls until both active backends are idle;
7. mark logging stopped.

No unbounded drain loop, `File.flush()`, FAT truncate, FAT close, or LittleFS
sync is executed from the GROUND transition.

The boot composition root prewarms both stores before sensor bring-up because
the current PCB has shown SDIO sensitivity to later bus initialization. The
task checks the already-open storage first and never repeats preallocation or
journal scanning during its own `init()`.

## 8. Record classes

### FAST, 20 ms

- FSM state and health flags
- low-g acceleration, gyro, R/P/Y and tilt
- high-g raw counts and acceleration
- barometer pressure, raw altitude and filtered altitude
- battery voltage and decision sequence

### SLOW, 100 ms

- magnetometer raw and converted values
- GPS position, altitude, speed, course, HDOP and satellites
- GPS parser counters
- barometer fault flags and counters

### EVENT

- boot and logger initialization
- every FSM transition
- storage fault
- final ground stop

### DECISION

- launch and burnout evidence
- apogee predictor quality and confirmation
- descent, tilt and timer fallback evidence
- main deployment and landing evidence

State transitions and decisions are source-queued. They are not inferred by
polling and therefore are not lost when multiple decisions occur in one logger
period.

## 9. Recovery tooling

`tools/decode_flight_log.py` auto-detects three inputs:

1. SD `NSDB` block file
2. raw W25Q128 `NSEC`/`NPAG` dump
3. plain logical `.NLG` frame stream

For SD it selects the current session and validates every block. For QSPI it
orders sectors and pages by sequence/stream offset and discards torn pages. It
then decodes only application frames with a valid frame CRC.

## 10. Implementation map

| Responsibility | File |
| --- | --- |
| Frame format and CRC | `src/logging/flight_log_record.*` |
| 16 KiB complete-record FIFO | `src/logging/flight_log_ram_buffer.*` |
| Per-backend 8 KiB byte queue | `src/logging/flight_log_byte_queue.*` |
| Async storage interface and mirror | `src/logging/flight_log_storage.h`, `flight_log_mirror_storage.*` |
| W25Q128 bounded FlexSPI command HAL | `src/hal/w25q128_qspi_hal.*` |
| QSPI raw circular journal | `src/logging/program_flash_flight_log_storage.*` |
| SDIO preallocated block journal | `src/logging/sd_flight_log_storage.*` |
| Producer and async GROUND drain | `src/missions/flight_log_task.*` |
| Host decoder | `tools/decode_flight_log.py` |
| Host tests | `test/logging/*` |
| Hardware end-to-end test | full `main`/`debug_no_lora` firmware path |

## 11. Verification and flight acceptance

Completed software checks:

- frame CRC and FIFO overflow tests
- backend byte-queue wrap test
- mirror degradation test
- SD and QSPI wrapper decoder tests, including corrupted payload rejection
- `main` firmware build
- `debug_no_lora` firmware build

Required on the final board and selected SD card:

1. Run `debug_no_lora`; validate SD block CRCs and QSPI page CRCs.
2. Measure every `flight_log` tick with microsecond resolution during QSPI erase,
   QSPI rollover, SD write, and SD busy intervals.
3. Proposed release gate: logger tick maximum below 5 ms. This leaves at least
   half of the 10 ms FSM period; it is an engineering acceptance proposal and
   must be confirmed with measured hardware results.
4. Run at twice nominal production rate for at least ten minutes with zero RAM
   record drops.
5. Remove or stall SD during logging and verify QSPI continues without a missed
   FSM/sensor period.
6. Force QSPI timeout and verify SD continues.
7. Cut power repeatedly during page program, sector erase, SD transfer and
   GROUND drain. The decoder must recover every complete block/page/frame before
   the torn unit and must not emit stale preallocated SD data.
8. Fill and wrap the complete QSPI journal and verify monotonically ordered
   recovery of the retained window.

This implementation is not flight-qualified until the hardware latency and
power-interruption tests above are recorded.
