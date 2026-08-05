# GPS Standalone Test

Independent Teensy 4.1 test for the avionics u-blox M6 UART and NMEA output.
It does not initialize flight state, pyro, storage, sensors, or LoRa.

```bash
pio run -d test/gps_standalone_test -e gps_standalone -t upload
pio device monitor -p /dev/ttyACM0 -b 115200
```

`RESULT GPS_NMEA_LINK PASS` proves that UART data and valid NMEA checksums are
working. `RESULT GPS_FIX PASS` additionally requires a usable satellite fix and
normally needs an outdoor sky view.
