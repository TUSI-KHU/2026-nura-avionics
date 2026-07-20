# SparkFun SPX-18572 / E19-915M30S SX1276 1W standalone test

This is a bench-only Teensy 4.1 test for the SparkFun LoRa 1W Breakout
(SPX-18572, E19-915M30S) with its external RX/TX RF switch.

The supplied wiring is:

| SX1276 breakout | Teensy 4.1 |
| --- | ---: |
| MISO | 1 (SPI1) |
| MOSI | 26 (SPI1) |
| SCK | 27 (SPI1) |
| NSS | 9 |
| RST | 24 |
| RXE | 30 |
| TXE | 31 |
| DIO0 | 32 |
| VCC | 5 V module rail (4.75-5.5 V recommended) |
| GND | GND |

`RXE` and `TXE` are RF-path enable signals, not UART receive/transmit pins. The
test assumes both are active high: receive is `RXE=1, TXE=0`, transmit is
`RXE=0, TXE=1`, and standby is `RXE=0, TXE=0`. Confirm that polarity against
the exact breakout schematic before transmitting.

The SparkFun/E19 module uses 5 V on its power input while its SPI/GPIO control
signals are 3.3 V logic. Keep the antenna connected and current-limit the
module rail during bring-up. The default RF profile is the repository
ground-radio profile: 920.9 MHz,
BW 125 kHz, SF7, CR 4/5, sync word `0x12`, CRC enabled, explicit header, and
2 dBm bench transmit power. Use a breakout and antenna matched to that band.

Build and upload from the repository root:

```sh
pio run -d test/sx1276_breakout
pio run -d test/sx1276_breakout -t upload
pio device monitor -d test/sx1276_breakout -b 115200
```

The sketch starts in continuous receive mode. Serial commands are:

- `t`: transmit one test packet and return to receive mode.
- `r`: enter receive mode.
- `q`: enter standby and disable both RF paths.
- `s`: print counters, state, and the RXE/TXE pin levels.
- `h`: print command help.

One radio alone can prove SPI communication, reset, DIO0 interrupt handling,
and RF-switch GPIO behavior. A real RF pass/fail test needs a second radio with
the same frequency, bandwidth, spreading factor, coding rate, sync word,
preamble, CRC, and header mode. Connect the correct antenna before issuing `t`.
