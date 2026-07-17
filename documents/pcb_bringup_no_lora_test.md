# PCB Bring-up No-LoRa Test

## Purpose

This branch is for first-pass PCB hardware bring-up only. It verifies Teensy
boot, LEDs, buzzer, no-LoRa sensors, GPS serial reception, battery voltage
sense, and manual bench-only pyro output pulses.

It is not a flight build.

## Inputs And Units

- LEDs: digital outputs on Teensy pins 34 and 33.
- Buzzer: digital/tone output on Teensy pin 2.
- MPL3115A2: I2C0, SDA 18, SCL 19, pressure in Pa.
- LIS3MDL: I2C1, SDA 17, SCL 16, magnetic field in uT.
- LSM6DSO32: SPI0, CS 10, acceleration in m/s^2, gyro in deg/s.
- H3LIS331DL: SPI0, CS 0, acceleration in g and m/s^2.
- GPS: Serial3, GPS TX to Teensy RX 15, GPS RX to Teensy TX 14, 9600 baud.
- Power sense: analog input 22, reported as raw ADC, sense mV, and battery mV.
- Pyro1: GPIO1 28, GPIO2 29, sense 25.
- Pyro2: GPIO1 38, GPIO2 35, sense 41.

## Allowed States

The test may run only on a bench with the PCB under direct human supervision.
Pyro output testing is allowed only when the output terminals are connected to
safe test loads or measurement equipment.

## Forbidden States

Do not use this firmware in a rocket, at a launch site, or with live
deployment charges connected. Do not treat a passing result as flight
acceptance.

## Thresholds And Sources

- GPS pass condition: at least one parsed NMEA checksum passes during the test
  window. This verifies electrical serial reception, not sky-view fix quality.
- Battery voltage valid range and divider scale come from
  `NuraConstants::Sensors`.
- MPL3115A2 pressure is checked against the datasheet range in
  `NuraConstants::MPL3115A2`.
- LSM6DSO32 and H3LIS331DL WHOAMI values come from the sensor datasheets and
  existing `NuraConstants`.
- Pyro pulse duration is 1000 ms, matching the existing flight constant scale,
  but this test path is manually armed and bench-only.

## Failure Modes Considered

- I2C bus not detecting the expected sensor address.
- SPI chip select or bus wiring returning the wrong WHOAMI.
- GPS UART swapped or silent.
- Battery sense divider absent, shorted, or out of expected voltage range.
- Pyro command typed accidentally.

## Fallback Behavior

Pyro outputs are off after initialization. A pyro pulse can only start after a
serial `ARM` command and the arm window expires after 10 seconds. `DISARM` and
`ALL_OFF` force both pyro channels off. Each `P1` or `P2` command produces one
1000 ms pulse and then turns outputs off.

## Verification Plan

1. Build with `pio run -e pcb_bringup_no_lora`.
2. Upload to a Teensy 4.1 mounted on the avionics PCB.
3. Open serial monitor at 115200 baud.
4. Confirm boot banner, pinmap, I2C scan output, each `TEST ... PASS/FAIL`
   line, and live sensor snapshots.
5. Confirm GPS electrical pass with antenna connected and module powered.
6. Confirm power sense against a bench supply or multimeter.
7. Connect safe dummy loads before pyro testing. Type `ARM`, then `P1` or `P2`,
   and confirm the expected terminal output and sense behavior.
