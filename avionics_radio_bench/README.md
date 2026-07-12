# Avionics SX1262 Bench Sender

Bench-only Teensy 4.1 firmware for validating the avionics PCB SX1262 radio
against the receive-only ground station. It must not configure or drive any
pyro, deployment, flight-state, sensor, or actuator pin.

Assumed PCB radio pin map:

```text
SPI1 MISO: D1
SPI1 MOSI: D26
SPI1 SCK:  D27
SX1262 NSS:   D9
SX1262 RXEN:  D30
SX1262 DIO1:  D31
SX1262 BUSY:  D32
```

The PCB designer identified the separate `LORA_RST JP2` exposed pads as the
manual radio reset. No MCU-controlled SX1262 `NRST` or `TXEN` connection is
currently documented.
This target is not flight firmware.

## Behavior

- `teensy41` initializes the SX1262 and prints status without transmitting.
- `tx` sends one authenticated FAST bench frame per second at 2 dBm.
- RXEN is held low during initialization and transmission.
- No pyro, deployment, flight-state, sensor, actuator, buzzer, or indicator pin
  is configured by either environment.

## Radio Settings

The initialization settings match the receive-only ground station: 920.9 MHz,
125 kHz bandwidth, spreading factor 7, coding rate 4/5, private sync word
`0x12`, eight preamble symbols, explicit header, normal IQ, and PHY CRC enabled.
The transmit target relies on the SX1262 DIO2-controlled RF switch configured by
RadioLib and holds the PCB RXEN pin low while transmitting. Output is limited to
2 dBm for this close-range bench test.
The SX1262 is initialized for an XTAL (`tcxoVoltage = 0 V`) because the default
TCXO configuration returned RadioLib `RADIOLIB_ERR_SPI_CMD_FAILED (-707)` on
the avionics PCB during the no-transmit bring-up test.

## Failure Modes And Verification

Initialization failure leaves the radio inactive and prints the RadioLib error
code. Frame encoding or transmission failure is logged and retried only on the
next one-second interval. The test must use an antenna or 50-ohm load and must
not be run with any pyro or deployment hardware energized.

Verification order:

1. Build and upload `teensy41`; require `SX1262_INIT_OK` without RF output.
2. Confirm the antenna or 50-ohm load before any RF output.
3. Build and upload `tx`; require increasing `TX_OK` sequence numbers.
4. Require decoded `rx type=FAST` lines on the receive-only ground station.
