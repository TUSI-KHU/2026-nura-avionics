# Sensor Defect Test Sketches

Each `.ino` file in this folder is a standalone bring-up sketch for one sensor/module.
Most files target Teensy 4.1. The LoRa sketch targets Arduino Nano by default.

Usage:

1. Open one `.ino` file at a time.
2. Edit the `PIN MAP / USER CONFIG` macros at the top of that file.
3. Upload to the target board named in the sketch.
4. Open Serial Monitor at `115200`.
5. Check `PASS`, `WARN`, and `FAIL` lines.

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
- `sx127x_lora_test.ino` - Arduino Nano default pin map, RA-01/SX1278 433 MHz default frequency.
