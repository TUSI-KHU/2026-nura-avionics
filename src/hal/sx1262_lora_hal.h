#pragma once

#include <stddef.h>
#include <stdint.h>

#include <RadioLib.h>

#include "board_pinmap.h"
#include "nura_constants.h"

struct Sx1262LoRaConfig
{
    long frequencyHz = NuraConstants::LoRa::kFlightFrequencyHz;
    int txPowerDbm = NuraConstants::LoRa::kFlightTxPowerDbm;
    int spreadingFactor = NuraConstants::LoRa::kSpreadingFactor;
    long signalBandwidthHz = NuraConstants::LoRa::kSignalBandwidthHz;
    int codingRateDenominator = NuraConstants::LoRa::kCodingRateDenominator;
    long preambleLength = NuraConstants::LoRa::kPreambleLength;
    int syncWord = NuraConstants::LoRa::kSyncWord;
    float tcxoVoltage = NuraConstants::LoRa::kFlightTcxoVoltage;
    bool useRegulatorLdo = NuraConstants::LoRa::kFlightUseRegulatorLdo;
    bool crcEnabled = true;
};

struct Sx1262LoRaPacket
{
    size_t length = 0U;
    int rssi = 0;
    float snr = 0.0f;
    long frequencyError = 0L;
};

class Sx1262LoRaHAL
{
public:
    bool begin(const Sx1262LoRaConfig &config);
    void end();

    bool send(const uint8_t *data, size_t length);
    bool receive(uint8_t *buffer, size_t capacity, Sx1262LoRaPacket &packet);
    int rssi();

private:
    bool applyConfig(const Sx1262LoRaConfig &config);
    bool startReceive();

    Module module_{BoardPinMap::Sx1262LoRa::ssPin,
                   BoardPinMap::Sx1262LoRa::dio1Pin,
                   RADIOLIB_NC,
                   BoardPinMap::Sx1262LoRa::busyPin,
                   SPI1,
                   SPISettings(NuraConstants::LoRa::kFlightSpiFrequencyHz,
                               MSBFIRST,
                               SPI_MODE0)};
    SX1262 radio_{&module_};
    bool initialized_ = false;
};
