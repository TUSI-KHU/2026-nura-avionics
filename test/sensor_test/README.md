# Sensor Defect Test Sketches

Each `.ino` file in this folder is a standalone bring-up sketch for one sensor/module.
The active bring-up sketches target Teensy 4.1.

Shared SPI wiring for H3LIS331DL, the low-g IMU, and LoRa:

- MOSI: 11
- MISO: 12
- SCK: 13
- Low-g IMU CS: 6
- H3LIS331DL CS: 7
- LoRa NSS/SS: 10
- LoRa RST: 9
- LoRa DIO0: 2

Usage:

1. Open one `.ino` file at a time.
2. Edit sensor pins in `include/board_pinmap.h` if the wiring changes.
3. Edit non-pin test settings in the `PIN MAP / USER CONFIG` block if needed.
4. Upload to the target board named in the sketch.
5. Open Serial Monitor at `115200`.
6. Check `PASS`, `WARN`, and `FAIL` lines.

These sketches are for breakout bring-up and defect screening before PCB integration.
They do not replace full calibration, environmental testing, or flight qualification.

Required Arduino libraries:

- Adafruit LSM6DS
- Adafruit LIS3MDL
- Adafruit LIS331
- Adafruit MPL3115A2 Library
- MS5611 by Rob Tillaart
- TinyGPSPlus
- LoRa by Sandeep Mistry

Files:

- `lsm6dsox_low_g_imu_test.ino`
- `lsm6dso32_low_g_imu_test.ino`
- `adxl377_high_g_accel_test.ino`
- `h3lis331dl_high_g_accel_test.ino`
- `lis3mdl_mag_test.ino`
- `ms5611_baro_test.ino`
- `mpl3115a2_baro_test.ino`
- `ublox_m6_gnss_test.ino`
- `sx127x_lora_test.ino` - Teensy SPI pin map, RA-01/SX1278 433 MHz default frequency.
