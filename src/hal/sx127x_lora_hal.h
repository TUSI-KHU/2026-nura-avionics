#pragma once

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>
#include <SPI.h>

#include "board_pinmap.h"
#include "nura_constants.h"

struct Sx127xLoRaConfig
{
    // SparkFun SPX-18572 / E19-915M30S is an SX1276 register-compatible 1 W
    // breakout. TX/RX power switching is handled through RXE/TXE below.
    long frequencyHz = NuraConstants::LoRa::kFlightFrequencyHz;
    int ssPin = BoardPinMap::SparkFunSx1276_1W::ssPin;
    int resetPin = BoardPinMap::SparkFunSx1276_1W::resetPin;
    int libraryResetPin = BoardPinMap::SparkFunSx1276_1W::libraryResetPin;
    int dio0Pin = BoardPinMap::SparkFunSx1276_1W::dio0Pin;
    int rxEnablePin = BoardPinMap::SparkFunSx1276_1W::rxEnablePin;
    int txEnablePin = BoardPinMap::SparkFunSx1276_1W::txEnablePin;
    bool rfSwitchActiveHigh = true;
    uint32_t spiFrequency = NuraConstants::LoRa::kFlightSpiFrequencyHz;
    uint8_t spiMode = NuraConstants::LoRa::kFlightSpiMode;
    bool probeSpiMode = false;
    uint8_t initAttempts = 1U;
    int txPowerDbm = NuraConstants::LoRa::kFlightTxPowerDbm;
    int spreadingFactor = 7;
    long signalBandwidthHz = 125000L;
    int codingRateDenominator = 5;
    long preambleLength = 8;
    int syncWord = 0x12;
    bool crcEnabled = true;
    bool downlinkOnly = NuraConstants::LoRa::kFlightDownlinkOnly;
};

struct Sx127xLoRaPacket
{
    size_t length = 0U;
    int rssi = 0;
    float snr = 0.0f;
    long frequencyError = 0L;
};

class Sx127xLoRaHAL
{
public:
    bool begin(const Sx127xLoRaConfig &config, SPIClass &spi = SPI);
    void end();

    bool send(const uint8_t *data, size_t length, bool async = false);
    void service(uint32_t nowMs);
    bool txBusy() const;
    bool receive(uint8_t *buffer, size_t capacity, Sx127xLoRaPacket &packet);
    int rssi() const;

private:
    bool applyConfig(const Sx127xLoRaConfig &config);
    bool selectSpiMode(const Sx127xLoRaConfig &config, SPIClass &spi);
    uint8_t readRegisterRaw(const Sx127xLoRaConfig &config, SPIClass &spi, uint8_t address, uint8_t spiMode);
    void resetRadio(const Sx127xLoRaConfig &config);
    void setRfPath(const Sx127xLoRaConfig &config, bool receivePath, bool transmitPath);

    bool initialized_ = false;
    bool txBusy_ = false;
    Sx127xLoRaConfig activeConfig_{};
    uint8_t selectedSpiMode_ = SPI_MODE0;
    uint32_t selectedSpiFrequency_ = 8000000UL;
};
