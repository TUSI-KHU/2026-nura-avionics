# Korea LoRa Bench Frequency Plan V1

Status: Bench frequency plan for Korean unlicensed RFID/USN-style operation

## Purpose

Keep the NURA SX1276 bench and flight-link test profiles inside the Korean
917-923.5 MHz RFID/USN channel plan while leaving conservative margin for output
power and antenna gain during ground tests.

## Selected Channel

- Center frequency: `922.3 MHz`
- Firmware value: `922300000 Hz`
- Channel: Korean RFID/USN channel 27
- LoRa bandwidth: `125 kHz`
- Modem profile: SF7, BW125, CR4/5, sync word `0x12`, CRC enabled

The center frequency is one of the published 200 kHz-spaced RFID/USN channel
centers in the 917-923.5 MHz band. At 125 kHz occupied bandwidth, the nominal
LoRa signal remains inside the 917-923.5 MHz allocation when centered at
922.3 MHz.

## Output Power Policy

- Flight-link default TX power: `2 dBm`
- Bench diagnostic TX power: `2 dBm`

The 922.3 MHz channel sits in the upper channel group that allows higher USN
radiated power than the lower 917 MHz channels. The firmware still uses a very
conservative 2 dBm radio setting because the legal limit is based on radiated
power including antenna gain, not only the software TX power value.

## Flight-Build Impact

The normal `main` build uses `922300000 Hz` and `2 dBm`. This change only sets
the RF profile. It does not change deployment, arming, state transition, pyro,
abort, or recovery logic.

## Remaining Compliance Checks

- Confirm the actual antenna gain and cable loss.
- Confirm measured conducted/radiated output power with the selected module.
- Confirm duty-cycle or LBT requirements for the final telemetry packet rate.
- Confirm any competition/site-specific frequency restrictions before flight.
