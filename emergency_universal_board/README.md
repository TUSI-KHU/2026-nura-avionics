# NURA Emergency Universal Board Firmware

Same-day fallback firmware for a hand-wired Teensy 4.1 stack.

## Hardware Assumptions

- MCU: Teensy 4.1.
- Storage: built-in SDIO microSD plus Teensy 4.1 underside QSPI flash.
- I2C sensors on `Wire`:
  - BMP390 at `0x77`.
  - BNO085 at `0x4A`.
- Pyro MOSFET outputs are active-high:
  - Drogue: pin `2`.
  - Main: pin `3`.
- Buzzer, LoRa, GNSS, magnetometer, and battery-voltage sensing are disabled in this fork.

If BNO085 appears at `0x4B`, change `BoardPinMap::BNO085::i2cAddress` in `include/board_pinmap.h`.
If BMP390 appears at `0x76`, change `BoardPinMap::BMP390::i2cAddress`.

## Build

```bash
pio run -d emergency_universal_board -e main
```

Firmware output:

```bash
emergency_universal_board/.pio/build/main/firmware.hex
```

Upload:

```bash
pio run -d emergency_universal_board -e main -t upload
```

I2C scanner:

```bash
pio run -d emergency_universal_board -e i2c_scanner -t upload
```

Host-side FSM replay, including OpenRocket-derived INIT-to-GROUND check:

```bash
python3 test/fsm_replay/run_fsm_replay_tests.py
```

## Kept From Main Firmware

- Existing scheduler structure.
- Existing flight state machine.
- Existing SD + QSPI flash mirrored flight logging.
- Existing MOSFET pyro output abstraction and active-high firing policy.
- Existing panic path and serial logger.

## Current Replay Result

The OpenRocket-derived nominal sustainer replay uses the evaluation dataset's `502.911 m` apogee at `10.500 s`.

Last checked transition times:

- SAFE: `0 ms`
- ARMED: `10 ms`
- LAUNCH: `1030 ms`
- COAST: `2580 ms`
- APOGEE: `10050 ms`
- DROGUE: `13050 ms`
- DEPLOY: `22800 ms`
- GROUND: `31800 ms`

## Same-Day Changes

- Replaced the old low-g/high-g IMU stack with one BNO085 task.
- The BNO085 sample is published to both `ImuState` and `HighGImuState` so the existing FSM acceleration paths keep working.
- Replaced the BMP180/BMP-series barometer HAL path with BMP390 through Adafruit BMP3XX.
- Removed runtime registration of LoRa, GNSS, magnetometer, and power sensing.

## Must-Do Bench Checks Before Flight

- Confirm I2C scan sees BMP390 and BNO085 at the addresses above.
- Confirm serial boot logs show `bno085 initialized` and `bmp390 initialized`.
- Confirm SD file creation and QSPI flash mount succeed.
- Confirm pins 2 and 3 are LOW after boot and only go HIGH during an intentional pyro bench test.
- Confirm BNO085 acceleration magnitude exceeds the launch threshold during a safe bench acceleration test.
- Confirm BMP390 altitude increases when pressure is reduced or the board is raised.

## Safety Notes

This is an emergency fallback, not the validated PCB stack. BNO085 is not a dedicated high-g launch accelerometer. The current FSM still uses the existing thresholds in `include/nura_constants.h`, so treat the BNO085 launch/burnout path as a same-day risk item until bench data confirms it crosses the required thresholds without saturating or filtering away the event.
