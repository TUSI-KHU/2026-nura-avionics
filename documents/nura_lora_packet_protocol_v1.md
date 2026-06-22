# NURA LoRa Packet Protocol V2 Authenticated Lite

Status: Draft for implementation  
Target vehicle: 2026 NURA avionics, sub-1 km class university rocket  
Target radios: flight SX1262 and ground SX1276, both using the same proprietary LoRa PHY profile at 920.9 MHz
Target controller: Teensy 4.1  

This document defines the authenticated NURA V2 Lite application-layer packet format carried inside a LoRa PHY packet. The filename is retained to avoid breaking existing repository links. V2 intentionally removes nonessential packet classes so the radio spends less time transmitting and more time available for uplink commands.

This document is not a legal certification or RF compliance report. Frequency, output power, antenna gain, and LBT behavior must be verified against the final competition and Korean radio requirements before flight.

## 1. Design Decision

V1 Lite uses only three active payload classes:

```text
FAST_TLM   high-rate flight telemetry
GPS_TLM    slow recovery/navigation telemetry
CONTROL    uplink command and downlink command response
```

Removed from active V1 Lite:

```text
STATUS_TLM
EVENT_LOG
HEARTBEAT
separate CMD packet type
separate ACK packet type
```

The removed information is not forbidden forever. It is deferred until range tests prove there is enough airtime. In V1 Lite, every nonessential byte is treated as a command-window tax.

## 2. Operating Model

Telemetry is a lossy stream:

- FAST_TLM is never retransmitted.
- GPS_TLM is never retransmitted.
- Sequence gaps are recorded for link-quality estimation only.
- The newest valid sample wins.

Control is reliable enough for emergency use:

- GCS retransmits CONTROL/CMD until it receives CONTROL/ACK or the command expires.
- Avionics deduplicates commands by `command_seq`.
- A duplicate command must never re-execute an actuator action.
- ACK has higher priority than telemetry.
- Avionics keeps the radio in receive mode whenever it is not transmitting.

Safety rule:

```text
A CONTROL/CMD requests an action. It must not directly overwrite the flight state.
```

For recovery deployment, the command path should verify safety conditions and inject a recovery action/event into the flight or recovery controller. The parser itself must never energize a pyro channel.

## 3. RF Constraints and Rate Target

Recommended flight RF profile:

| Parameter | Flight default | Notes |
| --- | --- | --- |
| Frequency | 920.9 MHz | Must be identical on flight SX1262 and ground SX1276. |
| Bandwidth | 125 kHz | Preferred for link margin and regional channel compatibility. |
| Spreading factor | SF7 | Increase SF only after reducing traffic rate. |
| Coding rate | CR 4/5 | Higher CR increases airtime. |
| Preamble | 8 symbols | Default unless range tests show a need to change. |
| Sync word | `0x12` | Must be identical on both radios. |
| LoRa PHY CRC | Enabled | Application CRC is still required. |
| Header mode | Explicit | Easier interoperability and debugging. |
| EIRP | Must not exceed applicable limit | Include antenna gain and cable loss. |

`frequency`, bandwidth, spreading factor, coding rate, preamble, sync word,
header mode, and PHY CRC are an interoperability contract: the flight SX1262
and ground SX1276 must use the same values. Transmit power is local to each
radio and is not required to match.

Flight SX1262 HAL wiring is SPI1 at 2 MHz (MISO D1, MOSI D26, SCK D27),
NSS D9, DIO1 D31, and BUSY D32. The 2 MHz host SPI clock was verified by the
2026-06-22 avionics radio bench test. The currently
recorded PCB data has no Teensy-controlled `NRESET`, so the driver uses
no-reset mode. Its `TCXO` setting is `0 V`, which assumes a crystal rather than
a DIO3-controlled TCXO. `RXE` D30 is intentionally left untouched until its
schematic role and polarity are confirmed. SX1262 initialization and two-way
packet exchange are mandatory hardware acceptance tests before this profile is
used outside the bench.

The experimental `debug_radio_bench` environment runs the real sensor and FSM
tasks while holding RXE D30 low behind `NURA_BENCH_SX1262_RXE_LOW`. Its purpose
is downlink telemetry integration on the bench; it also limits output to the
ground-tested 2 dBm through `NURA_BENCH_RADIO_TX_POWER_DBM`. It is forbidden
for flight and does not enable physical pyro outputs. It also defines
`NURA_BENCH_RADIO_DOWNLINK_ONLY`, so the HAL does not enter receive mode while
RXE is held low. The RXE LOW, 2 dBm, and downlink-only settings come from the
successful 2026-06-22 SX1262-to-SX1276 bench test.
Acceptance requires SX1262 initialization, authenticated FAST/GPS reception,
increasing sequence numbers, and zero decode failures. RXE polarity and
bidirectional switching must still be confirmed from the PCB schematic before
the flight build may drive this pin.

At SF7/BW125/CR4/5/preamble 8/CRC on/explicit header, approximate airtime is:

| V2 Lite frame | Frame size | Approx. airtime |
| --- | ---: | ---: |
| FAST_TLM | 41 B | 87.3 ms |
| GPS_TLM | 37 B | 82.2 ms |
| CONTROL | 43 B | 87.3 ms |

Nominal schedule:

```text
FAST_TLM: 5 Hz
GPS_TLM: 1 Hz
CONTROL: on demand, telemetry may be skipped
```

Short burst schedule:

```text
FAST_TLM: 8 Hz
GPS_TLM: 1 Hz
CONTROL: on demand, telemetry may be skipped
```

Do not treat 10 Hz as the flight default. It may work in benign bench conditions, but it consumes too much receive-window margin for emergency uplink.

If the link degrades:

```text
1. Reduce FAST_TLM to 2-3 Hz.
2. Keep GPS_TLM at 1 Hz if recovery is still needed.
3. If needed, switch to SF8 at lower rates.
4. After landing, use low-rate GPS beacon and optionally SF9/SF10.
```

## 4. Application Frame

V2 Lite uses fixed-length payloads selected by message type. There is no `payload_len` field. This reduces overhead and parser complexity.

Frame overhead: 19 bytes.

| Offset | Type | Field | Description |
| ---: | --- | --- | --- |
| 0 | u8 | sync0 | Fixed 0xAA |
| 1 | u8 | sync1 | Fixed 0x55 |
| 2 | u8 | ver_type | Upper nibble = version, lower nibble = message type |
| 3 | u32 | vehicle_id | Expected vehicle identity |
| 7 | u16 | seq | Direction-local sequence number |
| 9 | bytes | payload | Fixed length by message type |
| 9 + payload_len | u8[8] | frame_auth_tag | SipHash-2-4 over direction, header, and payload |
| 17 + payload_len | u16 | crc16 | CRC over ver_type through authentication tag |

`ver_type`:

```text
bits 7..4: protocol version = 2
bits 3..0: message type
```

Message types:

| Type | Name | Direction | Payload bytes | Frame bytes |
| ---: | --- | --- | ---: | ---: |
| 0x1 | FAST_TLM | Avionics to GCS | 22 | 41 |
| 0x2 | GPS_TLM | Avionics to GCS | 18 | 37 |
| 0x3 | CONTROL | Bidirectional | 24 | 43 |
| 0x4..0xF | Reserved | - | - | - |

All multi-byte integers are little-endian.

Do not transmit raw C++ structs directly. Encode each field explicitly.

## 5. CRC

Recommended CRC:

```text
Name: CRC-16/CCITT-FALSE
Polynomial: 0x1021
Initial value: 0xFFFF
RefIn: false
RefOut: false
XorOut: 0x0000
Coverage: ver_type, vehicle_id, seq, payload, frame_auth_tag
Excluded: sync0, sync1, crc16 field
```

Frames with CRC failure must be dropped. A CRC-failed CONTROL/CMD must not be ACKed because command identity and payload integrity are not trustworthy.

### 5.1 Frame Authentication

Every frame carries an 8-byte SipHash-2-4 tag. The authenticated input is:

```text
direction_domain, ver_type, vehicle_id, seq, payload
```

`direction_domain` is `0x55` for uplink and `0x44` for downlink. It is not
transmitted; each endpoint supplies the expected direction while verifying.
This prevents a valid downlink frame from being reflected into the uplink path.

The receiver must compare `vehicle_id` with its configured vehicle, verify the
tag with the shared 128-bit key, and reject the frame before payload dispatch if
either check fails. The public fallback ID/key in the repository are for bench
tests only. Flight builds require a gitignored `include/nura_radio_secrets.h`.

## 6. Packet Validation

Each LoRa PHY packet must carry exactly one complete application frame:

```text
READ_BOUNDED_PACKET -> CHECK_LENGTH/TYPE -> CHECK_CRC -> CHECK_ID/MAC -> DISPATCH
```

Rules:

- Unknown version is a parse error.
- Unknown message type is rejected.
- Payload length is determined only by message type.
- Short, long, concatenated, and oversized PHY packets are rejected.
- The parser must not allocate dynamic memory.
- The radio FIFO must still be drained when the packet exceeds the application
  buffer, without writing past that buffer.

Maximum V2 Lite frame size:

```text
NURA_V2_MAX_FRAME_LEN = 43 bytes
```

## 7. Fixed-Point Encoding

Floating-point values are not transmitted in V2 Lite.

Signed invalid sentinel:

```text
i8 invalid  = -128
i16 invalid = -32768
i32 invalid = -2147483648
```

Unsigned invalid sentinel:

```text
u8 invalid  = 0xFF
u16 invalid = 0xFFFF
u32 invalid = 0xFFFFFFFF
```

Use saturating encode. If a value exceeds the valid range, clamp to the nearest valid non-sentinel value and clear the relevant status bit if the value is not trustworthy.

## 8. FAST_TLM Payload

FAST_TLM carries the minimum high-rate flight state.

Payload length: 22 bytes  
Frame length: 41 bytes

| Offset | Type | Field | Unit / encoding |
| ---: | --- | --- | --- |
| 0 | u16 | status_word | Sensor health and flight state bitfield |
| 2 | u32 | boot_ms | Milliseconds since avionics boot |
| 6 | i16 | baro_dp_2pa | `(pressure_pa - ground_pressure_pa) / 2` |
| 8 | i16 | low_accel_x_cg | Low-g acceleration, `g * 100` |
| 10 | i16 | low_accel_y_cg | Low-g acceleration, `g * 100` |
| 12 | i16 | low_accel_z_cg | Low-g acceleration, `g * 100` |
| 14 | i16 | gyro_x_dps10 | Gyro, `deg/s * 10` |
| 16 | i16 | gyro_y_dps10 | Gyro, `deg/s * 10` |
| 18 | i16 | gyro_z_dps10 | Gyro, `deg/s * 10` |
| 20 | u16 | batt_mv | Battery voltage in mV. `0` means unavailable or invalid. |

`batt_mv` is intentionally kept as a raw unsigned millivolt value instead of a
floating-point voltage. The avionics firmware currently derives it from the
battery divider with a measured division ratio of 5.545:

```text
sense_mv   = raw_adc / 1023 * 3300
batt_mv    = sense_mv * 5545 / 1000
```

This preserves 1 mV packet resolution while keeping the FAST payload at 22
bytes. The field is not a flight-safety decision input in V1 Lite; if the ADC
reading is stale or outside the sanity range, avionics sends `0`.

Implementation notes for V1 Lite:

- Purpose: ground visibility of the avionics battery pack voltage.
- Source: D21 analog voltage divider. The final Pyro 1 outputs are D28/D29,
  so the voltage input does not share a pyro output pin.
- Calibration source: team board measurement/calculation: divider ratio 5.545;
  12.6 V maps to 2.2723 V (about raw ADC 704) and 11.1 V maps to 2.0018 V
  (about raw ADC 620) on a 10-bit, 3.3 V ADC.
- Sanity range: firmware accepts 6.0 V to 14.0 V as a valid electrical reading.
  The 11.1 V to 12.6 V values above are the expected 3S operating range, not a
  state-machine or pyro threshold.
- Allowed states: all states, because this is telemetry-only.
- Forbidden use: do not use `batt_mv` as a pyro, arming, or state-transition
  gate until the power hardware and threshold policy are separately validated.
- Fallback behavior: invalid, stale, or unavailable samples downlink as `0`.
- Verification: bench-feed the divider with known voltages, confirm receiver
  `batt_mv` printout and local log `battery_mv` match the expected pack voltage.

### 8.1 FAST Status Word

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | LOW_IMU_OK | Low-g IMU sample valid |
| 1 | HIGH_ACCEL_OK | High-g accelerometer alive/valid, even if not included in FAST |
| 2 | BARO_OK | Barometer sample valid |
| 3 | GPS_OK | GNSS parser alive and last GPS not stale |
| 4 | MAG_OK | Magnetometer alive/valid |
| 5 | STORAGE_OK | Local logging/storage healthy |
| 6 | PYRO_CONTINUITY_OK | Recovery output continuity healthy, if measured |
| 7 | RADIO_OK | Radio HAL initialized and not faulted |
| 8..11 | FLIGHT_STATE | Current state-machine state, 0..15 |
| 12 | ARMED | Avionics is armed |
| 13 | ABORT_ACTIVE | Abort/failsafe state is active |
| 14 | DEPLOY_FIRED | Recovery deployment action has been commanded/executed |
| 15 | CRITICAL_FAULT | Flight-critical fault is active |

State encoding:

| Value | State |
| ---: | --- |
| 0 | INIT |
| 1 | SAFE |
| 2 | ARMED |
| 3 | LAUNCH |
| 4 | COAST |
| 5 | APOGEE |
| 6 | DROGUE |
| 7 | DEPLOY |
| 8 | GROUND |
| 9 | FAULT |
| 10..15 | Reserved |

### 8.2 Pressure Encoding

`baro_dp_2pa` is relative to the pad baseline:

```text
baro_dp_2pa = round((pressure_pa - ground_pressure_pa) / 2)
pressure_pa = ground_pressure_pa + (baro_dp_2pa * 2)
```

This provides 2 Pa pressure resolution with an i16 range of approximately +/-65.5 kPa. For a sub-1 km rocket, this is far wider than expected pressure change.

GCS must learn `ground_pressure_pa` before launch. Because STATUS_TLM was removed from V1 Lite, use one of these methods:

```text
Preferred: GCS records ground_pressure_pa during pad setup from serial/USB config.
Acceptable: first valid pre-arm FAST_TLM packets define the GCS pressure baseline.
Fallback: GCS displays baro_dp_2pa and GPS altitude without absolute pressure reconstruction.
```

High-g acceleration is intentionally not included in FAST_TLM V1 Lite. The high-g sensor health is still reported in `HIGH_ACCEL_OK`. If flight tests show that high-g downlink is essential, add one 2-byte high-g magnitude field only after re-running airtime and command-latency tests.

## 9. GPS_TLM Payload

GPS_TLM carries recovery-critical location at slow rate.

Payload length: 18 bytes  
Frame length: 37 bytes

| Offset | Type | Field | Unit / encoding |
| ---: | --- | --- | --- |
| 0 | i32 | latitude_e7 | Latitude degrees * 10,000,000 |
| 4 | i32 | longitude_e7 | Longitude degrees * 10,000,000 |
| 8 | i16 | gps_alt_dm | GPS altitude, meters * 10 |
| 10 | u16 | speed_cms | Ground speed, m/s * 100 |
| 12 | u16 | course_cdeg | Course over ground, degrees * 100 |
| 14 | u8 | hdop_x10 | HDOP * 10, saturated |
| 15 | u8 | satellites | Satellite count |
| 16 | u8 | fix_flags | GPS fix bitfield |
| 17 | u8 | age_100ms | Location age in 100 ms units, saturated |

GPS_TLM does not include `boot_ms` in V1 Lite. The receiver uses frame sequence and receive time. FAST_TLM already carries avionics time at higher rate.

### 9.1 GPS Fix Flags

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | HAS_FIX | Location is valid and not stale |
| 1 | LOCATION_VALID | GNSS library reports valid lat/lon |
| 2 | ALTITUDE_VALID | GNSS library reports valid altitude |
| 3 | SPEED_VALID | GNSS library reports valid speed |
| 4 | COURSE_VALID | GNSS library reports valid course |
| 5 | FIX_STALE | Location age exceeds configured freshness limit |
| 6..7 | FIX_TYPE | 0 unknown, 1 2D, 2 3D, 3 reserved |

Transmit GPS_TLM at the scheduled slow rate even when there is no fix. Use invalid sentinels for unavailable fields and clear `HAS_FIX`.

## 10. CONTROL Payload

CONTROL is a fixed-length union packet. It carries both uplink commands and downlink ACKs.

Payload length: 24 bytes  
Frame length: 43 bytes

| Offset | Type | Field | CMD meaning | ACK meaning |
| ---: | --- | --- | --- | --- |
| 0 | u8 | subtype | 0x01 CMD | 0x81 ACK |
| 1 | u8 | command_id | Command ID | Echo command ID |
| 2 | u16 | command_seq | Logical command sequence | Echo command sequence |
| 4 | u32 | nonce | GCS nonce | Echo nonce if available, else 0 |
| 8 | u32 | valid_until_ms | Command expiry/deadline | ACK timestamp `ack_boot_ms` |
| 12 | i16 | param0 | Command parameter 0 | Result detail 0 |
| 14 | i16 | param1 | Command parameter 1 | Result detail 1 |
| 16 | u8[8] | auth_or_ack | CMD auth tag | ACK stage/result/reason/state/detail |

### 10.1 CONTROL Subtypes

| Value | Name | Direction |
| ---: | --- | --- |
| 0x01 | CMD | GCS to avionics |
| 0x81 | ACK | Avionics to GCS |

Unknown CONTROL subtypes must be ignored after CRC validation.

### 10.2 Command IDs

Use one byte for command ID in V1 Lite.

| ID | Name | Required behavior |
| ---: | --- | --- |
| 0x01 | FORCE_DEPLOY_RECOVERY | Request recovery deployment through the safety-controlled recovery action path. |
| 0x02 | ABORT_PROPULSION_DEPRECATED | Deprecated/no-op; must return NOT_SUPPORTED or DEPRECATED and must not actuate hardware. |
| 0x03 | SET_TELEMETRY_PROFILE | Optional; change downlink rate/profile within allowed bounds. |
| 0x04..0x7F | Reserved for NURA |
| 0x80..0xFF | Experimental; must not be enabled in flight builds |

### 10.3 FORCE_DEPLOY_RECOVERY Parameters

`param0` selects the recovery channel:

| param0 | Meaning |
| ---: | --- |
| 0 | Default/single recovery channel |
| 1 | Drogue, if present |
| 2 | Main, if present |
| 3 | All recovery channels, only if explicitly supported |

For the current single-parachute system, accept only `param0 = 0`.

Implementation requirements:

- Verify application CRC.
- Verify authentication tag.
- Verify freshness/expiry.
- Check arming and safety interlocks.
- Deduplicate by `command_seq` and `nonce`.
- Route accepted command to the recovery controller or state-machine event path.
- Send CONTROL/ACK before and/or after action execution.

Implementation prohibitions:

- Do not treat command as raw state assignment.
- Do not re-execute duplicate commands.
- Do not use deprecated propulsion abort as an actuator command.
- Do not keep flight authentication keys in public source files.

### 10.4 Command Authentication

Safety-critical CONTROL/CMD packets must use the 8-byte `auth_or_ack` field as a truncated authentication tag.

Recommended construction:

```text
SipHash-2-4 with a 128-bit pre-shared key
or
HMAC-SHA256 truncated to 8 bytes
```

Authenticated bytes:

```text
ver_type, seq, subtype, command_id, command_seq,
nonce, valid_until_ms, param0, param1
```

Excluded bytes:

```text
sync bytes
crc16 field
auth_or_ack itself
```

GCS retransmits the exact same logical command with the same `command_seq`, `nonce`, and auth tag. A new nonce means a new logical command.

### 10.5 ACK Encoding in auth_or_ack

For CONTROL/ACK, `auth_or_ack[0..7]` is:

| Byte | Field | Meaning |
| ---: | --- | --- |
| 0 | ack_stage | Processing stage |
| 1 | result | Result code |
| 2 | reject_reason | Rejection reason |
| 3 | state_after | Flight state after processing |
| 4..5 | detail_flags | Little-endian u16 |
| 6..7 | reserved | Transmit 0 |

ACK stages:

| Value | Name | Meaning |
| ---: | --- | --- |
| 0 | RECEIVED | CMD syntax, CRC, and command ID parsed |
| 1 | ACCEPTED | CMD passed validation and is queued |
| 2 | EXECUTED | Requested action completed or reached commanded state |
| 3 | REJECTED | CMD rejected and will not execute |
| 4 | DUPLICATE | CMD already processed; no re-execution |

Result codes:

| Value | Name |
| ---: | --- |
| 0 | OK |
| 1 | AUTH_FAILED |
| 2 | EXPIRED |
| 3 | BAD_FORMAT |
| 4 | BAD_STATE |
| 5 | NOT_ARMED |
| 6 | ALREADY_DONE |
| 7 | NOT_SUPPORTED |
| 8 | ACTUATOR_FAULT |
| 9 | INTERNAL_ERROR |

Reject reasons:

| Value | Name |
| ---: | --- |
| 0 | NONE |
| 1 | COMMAND_EXPIRED |
| 2 | UNKNOWN_COMMAND |
| 3 | AUTH_TAG_MISMATCH |
| 4 | DUPLICATE_OLDER_COMMAND |
| 5 | DEPLOYMENT_INHIBITED |
| 6 | CONTINUITY_BAD |
| 7 | STATE_REJECTED |
| 8 | DEPRECATED_COMMAND |
| 9 | PROFILE_REJECTED |

For FORCE_DEPLOY_RECOVERY, avionics should send ACK/ACCEPTED quickly after validation and ACK/EXECUTED after the recovery action path reports completion. If only one ACK can be sent, ACK/EXECUTED is preferred after action completion.

## 11. Scheduling and Priorities

Priority order:

```text
1. RX handling for CONTROL/CMD
2. CONTROL/ACK transmission
3. FAST_TLM
4. GPS_TLM
```

Telemetry is skippable. ACK is not skippable unless the radio is faulted.

Recommended scheduler behavior:

- Do not queue stale FAST_TLM. Generate the latest sample when scheduled.
- If CONTROL/CMD is received, skip the next telemetry slot if needed and send CONTROL/ACK first.
- Keep LoRa in RX mode between downlink transmissions.
- Do not run 8 Hz burst if command ACK latency exceeds the requirement.
- Treat command latency tests as a release gate.

GCS command retry rule:

```text
Send CONTROL/CMD.
Wait 150-250 ms for CONTROL/ACK.
If no ACK, retransmit exact same CONTROL/CMD.
Stop on ACK/EXECUTED or ACK/REJECTED.
Expire after command deadline or operator-configured max retry window.
```

## 12. Sender Requirements

The avionics sender must:

- Encode fields explicitly as little-endian bytes.
- Maintain downlink sequence for FAST_TLM, GPS_TLM, and CONTROL/ACK.
- Keep a dedup cache for recent command sequence/nonce pairs.
- Keep the radio in receive mode whenever not actively transmitting.
- Skip telemetry when ACK must be sent.
- Source FAST status bits from current subsystem health.
- Source GPS_TLM from the latest GNSS parser state.
- Log richer raw data locally to SPI/program flash as the primary recorder, with microSD as a mirror if available; do not push it over V1 Lite LoRa.

## 13. Receiver/GCS Requirements

The receiver/GCS must:

- Decode the raw serial stream from the dumb gateway.
- Verify application CRC before dispatch.
- Decode fixed-length payload by message type.
- Track sequence gaps separately for FAST_TLM, GPS_TLM, and CONTROL.
- Display FAST health/state from `status_word`.
- Display GPS age and fix validity independently from FAST.
- Convert fixed-point fields back to engineering units.
- Retry CONTROL/CMD until CONTROL/ACK or timeout.
- Treat ACK/EXECUTED for FORCE_DEPLOY_RECOVERY as final success.
- Persist decoded telemetry to CSV or another analysis format after decoding.

## 14. Test Vectors

Minimum golden vectors:

- FAST_TLM nominal frame.
- FAST_TLM with invalid barometer and GPS status bits cleared.
- GPS_TLM no-fix frame.
- GPS_TLM valid-fix frame.
- CONTROL/CMD FORCE_DEPLOY_RECOVERY with valid auth tag.
- CONTROL/ACK ACCEPTED.
- CONTROL/ACK EXECUTED.
- CONTROL/ACK REJECTED for deprecated propulsion abort.
- CRC failure frame.
- Unknown message type.
- Garbage bytes before a valid sync.

## 15. Implementation Phases

Phase 1: Shared protocol library

- CRC16.
- Endian encode/decode helpers.
- Fixed-length frame encoder.
- Fixed-length parser state machine.
- FAST/GPS/CONTROL payload encode/decode.
- Golden-vector tests.

Phase 2: Sender

- FAST_TLM from status, pressure delta, low-g acceleration, gyro, battery.
- GPS_TLM from GNSS HAL.
- CONTROL/CMD receive and auth verification.
- CONTROL/ACK transmit.
- Scheduler with telemetry skip for ACK.

Phase 3: Receiver/GCS

- Serial stream parser.
- FAST/GPS display and CSV logging.
- CONTROL/CMD builder and retry loop.
- ACK state machine.
- Link statistics.

Phase 4: Flight-like validation

- Packet receive rate at 5 Hz and 8 Hz.
- Command ACK latency p50/p95/max.
- FORCE_DEPLOY_RECOVERY duplicate command test.
- Deprecated propulsion abort rejection test.
- GPS beacon after landing profile.

## 16. References

- Semtech SX1276/77/78/79 datasheet: LoRa packet duration and modem parameters. https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276
- Semtech SX1262 product information: 150-960 MHz LoRa transceiver and +22 dBm maximum output capability. https://www.semtech.com/products/wireless-rf/lora-connect/sx1262
- Semtech LoRa FAQ: SF and bandwidth trade off sensitivity/robustness against time-on-air. https://www.semtech.com/design-support/faq/faq-lora
- LoRa Alliance RP002 Regional Parameters, KR920-923 channel plan and data-rate definitions. https://read.uberflip.com/i/1540208/24
- The Things Network regional parameter summary, South Korea KR920-923. https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/other/
- Altus Metrum AltOS telemetry design: fixed-size rocket telemetry packet classes and separate sensor/GPS rates. https://altusmetrum.org/AltOS/doc/telemetry.html
- MAVLink serialization and command protocol: sequence, message ID, CRC, ACK/retry model, and signing concepts. https://mavlink.io/en/guide/serialization.html and https://mavlink.io/en/services/command.html
