# NURA Flight Build: No LoRa / No GPS / No LIS3MDL

This is a standalone PlatformIO copy for flying the current PCB with:

- LoRa disabled
- GPS disabled
- LIS3MDL magnetometer disabled because the board is populated without it
- LSM6DSO32, H3LIS331DL, MPL3115A2, battery sensing, pyro, buzzer, SD logging, and program/QSPI flash logging kept active

Build:

```bash
pio run -e main
```

Upload:

```bash
pio run -e main -t upload
```

Debug build:

```bash
pio run -e debug
```

Important pre-flight notes:

- Do not connect pyro charges during first power-on and init tests.
- `Pyro1` is drogue: GPIO1 D28, GPIO2 D29, sense D25.
- `Pyro2` is main: GPIO1 D37, GPIO2 D36, sense D40.
- LoRa pins are not initialized or driven in this build.
- GPS UART is not initialized in this build.
- LIS3MDL is not initialized in this build.
