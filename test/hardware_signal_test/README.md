# Hardware Signal Test

Standalone raw-bus diagnostic for the assembled avionics PCB.

- H3LIS331DL and LSM6DSO32: SPI0 identity stress and register write/readback
- MPL3115A2: I2C0 identity stress, idle levels, and register write/readback
- E19-915M30S/SX1276: SPI1 identity matrix and RF-switch GPIO test, with no RF transmission
- u-blox GNSS: UART receive validation and read-only UBX MON-VER poll

Pyro pins remain inputs with pulldowns. This project does not link the flight
application, sensor HALs, mission logic, storage, or telemetry tasks.

Send `r` over USB serial to repeat the complete test.
