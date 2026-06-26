#pragma once

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>
#include <SPI.h>

#include "board_pinmap.h"

struct Sx127xLoRaConfig
{
    // RFM95W/RFM96W and RA-01 share the Semtech SX127x register-level family.
    long frequencyHz = 915000000L;
    int ssPin = BoardPinMap::Ra01DevelopmentLoRa::ssPin;
    int resetPin = BoardPinMap::Ra01DevelopmentLoRa::resetPin;
    int libraryResetPin = BoardPinMap::Ra01DevelopmentLoRa::libraryResetPin;
    int dio0Pin = BoardPinMap::Ra01DevelopmentLoRa::dio0Pin;
    uint32_t spiFrequency = 8000000UL;
    uint8_t spiMode = SPI_MODE0;
    bool probeSpiMode = false;
    uint8_t initAttempts = 1U;
    int txPowerDbm = 17;
    int spreadingFactor = 7;
    long signalBandwidthHz = 125000L;
    int codingRateDenominator = 5;
    long preambleLength = 8;
    int syncWord = 0x12;
    bool crcEnabled = true;
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
    bool receive(uint8_t *buffer, size_t capacity, Sx127xLoRaPacket &packet);
    int rssi() const;

private:
    bool applyConfig(const Sx127xLoRaConfig &config);
    bool selectSpiMode(const Sx127xLoRaConfig &config, SPIClass &spi);
    uint8_t readRegisterRaw(const Sx127xLoRaConfig &config, SPIClass &spi, uint8_t address, uint8_t spiMode);
    void resetRadio(const Sx127xLoRaConfig &config);

    bool initialized_ = false;
    uint8_t selectedSpiMode_ = SPI_MODE0;
    uint32_t selectedSpiFrequency_ = 8000000UL;
};
