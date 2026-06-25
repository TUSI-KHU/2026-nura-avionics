# Sensors and Flight Hardware

This document tracks the sensor and support hardware planned for the 2026 NURA avionics stack, plus the firmware requirements needed before flight use.

Target controller: **Teensy 4.1**

> Note: This is a project hardware inventory and firmware integration checklist, not an electrical sign-off document. Confirm pinout, voltage level, bus address, connector orientation, and datasheet limits before PCB layout or launch hardware assembly.

## Sensor Requirements

| Device | Flight Role | Required Startup Checks | Calibration Requirement | Missing Calibration / Fault Impact |
| --- | --- | --- | --- | --- |
| MPL3115A2 | Barometer candidate / alternate altitude source | WHO_AM_I, mode configuration, pressure and temperature sanity range | Build ground pressure/altitude baseline on the pad before arming. If absolute altitude is required, set current field elevation or sea-level pressure. | Critical only if selected as the active altitude source. Relative altitude will be wrong if baseline is missing or stale. |
| LSM6DSO32 / LSM6DSOX | Low-g IMU, 16 g acceleration and gyro | WHO_AM_I, selected accel/gyro ranges, data-ready/read sanity, axis sign sanity | Auto-calibrate stationary accel and gyro bias at boot while the rocket is still on the pad. Confirm axis mapping before flight. | High impact. Gyro bias drift and accel offset can break attitude estimates, launch detection, and state transitions. |
| H3LIS331DL | High-g accelerometer candidate, 200 g class | WHO_AM_I, selected high-g range, sample sanity, axis sign sanity | Auto-calibrate stationary zero-g offset at boot. Confirm axis mapping before flight. | High impact if used for launch/high-g event detection or acceleration logging. |
| LIS3MDL | Magnetometer | WHO_AM_I, magnetic range sanity, saturation check | Hard-iron and soft-iron calibration must be done manually after assembly or stored from a prior calibration. Not a full boot-only calibration. | Medium impact unless heading is flight-critical. Bad calibration can make heading unusable near motors, batteries, or steel hardware. |
| u-blox M6 / NEO-6M | GNSS / position and time | UART response, baud rate, valid NMEA/UBX frames, fix status | No sensor calibration. Configure update rate, message set, and fallback behavior before flight. | Usually non-critical to autonomous recovery logic, but loss of telemetry position hurts tracking and recovery. |
| SX1262 | Flight LoRa telemetry | SPI command response, BUSY behavior, DIO1 IRQ, frequency band, output power, packet loopback/range test | No sensor calibration. RF settings and antenna match must be verified before launch. | Not a sensor fault, but telemetry loss can block ground visibility and recovery workflow. |

## Calibration Timing

| Timing | Devices | Requirement |
| --- | --- | --- |
| Automatic at boot, before arming | MPL3115A2, LSM6DSO32 / LSM6DSOX, H3LIS331DL | Run only while the rocket is stationary on the pad. Average multiple samples, reject obvious outliers, and fail arming if the active flight-critical sensor cannot calibrate. |
| Manual after boot or after assembly | LIS3MDL, optional barometer absolute-altitude reference | Rotate the fully assembled avionics stack for magnetometer calibration. Enter field elevation or sea-level pressure only if absolute altitude is needed. |
| Pre-flight characterization / stored constants | LIS3MDL, axis mapping for every motion sensor | Store board-specific magnetometer compensation and axis orientation after bench testing. Recheck after PCB, wiring, battery, or mounting changes. |

## Selected Flight Stack

| Role | Selected / Target Part | Form Factor | Notes |
| --- | --- | --- | --- |
| Controller | Teensy 4.1 | Board | Main flight controller target. |
| Low-g IMU, 16 g | LSM6DSOX / LSM6DSO32 | Breakout | 6-DOF accelerometer + gyroscope. BOM entry includes Adafruit LSM6DSO32 6-DOF accelerometer breakout. |
| Magnetometer | LIS3MDL | Breakout | STEMMA QT LIS3MDL magnetometer. |
| Pressure / barometer | MPL3115A2 | Breakout | Active pressure sensor for altitude estimation. |
| GNSS | u-blox M6 / NEO-6M | Breakout | Development module listed as GY-GPS6MV2 / NEO-6M GPS module. |
| LoRa radio | SX1262 | PCB | Target flight LoRa telemetry radio. |

## Flight Sensor Stack

| Role | Part | Form Factor | Description / Notes |
| --- | --- | --- | --- |
| Low-g IMU | LSM6DSOX / LSM6DSO32 | Breakout | Primary 16 g accelerometer + gyroscope path. |
| High-g accelerometer | H3LIS331DL | Breakout | Digital triple-axis accelerometer path for high-g launch detection and logging. |
| Magnetometer | LIS3MDL | Breakout | Magnetic heading source, subject to hard-iron and soft-iron calibration. |
| Pressure / barometer | MPL3115A2 | Breakout | Active pressure/altitude sensor. |
| GNSS | u-blox M6 / NEO-6M | Breakout | Position/time source for development and recovery. |
| LoRa radio | SX1262 | PCB | Flight transmitter at 920.9 MHz; shares the LoRa PHY profile with ground SX1276. |
| LoRa radio, ground | SX1276 | Module | Ground receiver at 920.9 MHz. |
| Battery voltage sense | 3S pack divider | PCB analog input | Telemetry-only pack voltage monitor on D21. Divider ratio is 5.545: 12.6 V maps to 2.2723 V and 11.1 V maps to 2.0018 V at the ADC input. |

## RF and Cabling

| Role | Part | Type | Description / Notes |
| --- | --- | --- | --- |
| U.FL to SMA cable | 1568-18568-ND | Cable | Coax cable, U.FL to SMA, 5.9 inch. |
| LoRa antenna | Generic SMA 915 MHz antenna | Antenna | SMA antenna listed as Wi-Fi HaLow / 915 MHz antenna. Match antenna band to the selected LoRa module and local competition rules. |

## Pyro / Driver and Passive Parts

| Role | Part | Form Factor | Description / Notes |
| --- | --- | --- | --- |
| MOSFET driver | TC4452 | THT | 12 A high-speed MOSFET driver. |
| Resistor | 100k resistor | THT | 100 kOhm resistor. |
| Resistor | 100 resistor | THT | 100 Ohm resistor. |
| Resistor | 22 resistor | THT | 22 Ohm resistor. |
| Capacitor | 0.1uF capacitor | THT | 0.1 uF capacitor. |
| Capacitor | 0.22uF capacitor | THT | 0.22 uF capacitor. |

## Expected Firmware Interfaces

| Device Group | Expected Interface | Firmware Status |
| --- | --- | --- |
| LSM6DSOX / LSM6DSO32 low-g IMU | I2C or SPI | HAL present. Sensor task integration still needed for the selected part. |
| H3LIS331DL high-g accelerometer candidate | I2C or SPI | HAL present for candidate testing. Select or drop before flight integration. |
| LIS3MDL magnetometer | I2C | HAL present. Manual calibration storage path still needed. |
| MPL3115A2 pressure sensor | I2C | HAL forces BAR/OS1 and uses non-blocking one-shot polling with a 50 ms conversion timeout for scheduler-safe 50 ms sampling. |
| u-blox M6 / NEO-6M GNSS | UART | HAL/parser scaffold present. Flight task and message configuration still needed. |
| SX1262 LoRa | SPI + DIO1/BUSY | HAL is integrated with RadioLib. `RXE` D30 and TCXO/reset hardware assumptions need bench confirmation. |
| SX1276 ground LoRa | SPI + DIO0 | Receiver uses the matching 920.9 MHz LoRa PHY profile. |
| TC4452 MOSFET driver | GPIO output | MOSFET pyro HAL present; physical output is build-gated by `NURA_ENABLE_PYRO_OUTPUTS`. Drogue/Pyro 1 is D28/D29. Main/Pyro 2 D37/D36 is not valid with built-in microSD because D34-D39 are SDIO-related; reassign before enabling main pyro output. |
| Battery voltage divider | Analog input | HAL and sensor task present. Publishes `TelemetryState.power.batteryMv` and FAST `batt_mv`; invalid/stale samples downlink as `0`. D21 uses a 10-bit, 3.3 V ADC and a divider ratio of 5.545. |

## Integration Notes

- Keep flight-critical sensor reads behind HAL classes under `src/hal/`.
- Add sensor acquisition tasks under `src/sensors/` and register them from `FlightControllerApp`.
- Store latest sensor values in small state structs under `src/state/`.
- Route recoverable sensor failures through `RecoverableTask` and `WatchdogTask` where appropriate.
- Keep watchdog recovery attempts bounded to one immediate sensor `begin()` attempt per recovery interval. Runtime recovery must not run multi-attempt `delay()` loops inside the cooperative scheduler.
- Keep board constants, bus addresses, task periods, and retry limits in `DefaultAppConfig`.
- Treat calibration failure on the active barometer or active IMU as an arming blocker.

## Removed / Not Used

| Device | Removal Reason |
| --- | --- |
| ADXL377 | Not populated in the current avionics stack. Its former analog pins overlap GPS and I2C1 nets, so the HAL and bring-up sketch were removed to avoid accidental enablement. |
| MS5611 | Not populated in the current avionics stack. MPL3115A2 is the active barometer, so the MS5611 HAL and bring-up sketch were removed. |
