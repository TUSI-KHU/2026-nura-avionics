#pragma once

#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <vector>

#include "Arduino.h"
#include "board_pinmap.h"
#include "nura_constants.h"

struct Sx127xLoRaConfig
{
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
    int spreadingFactor = NuraConstants::LoRa::kSpreadingFactor;
    long signalBandwidthHz = NuraConstants::LoRa::kSignalBandwidthHz;
    int codingRateDenominator = NuraConstants::LoRa::kCodingRateDenominator;
    long preambleLength = NuraConstants::LoRa::kPreambleLength;
    int syncWord = NuraConstants::LoRa::kSyncWord;
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
    bool begin(const Sx127xLoRaConfig &config, SPIClass &spi = SPI1)
    {
        (void)spi;
        lastConfig = config;
        initialized = beginResult;
        return initialized;
    }

    void service(uint32_t nowMs)
    {
        (void)nowMs;
    }

    bool txBusy() const
    {
        return false;
    }

    bool receive(uint8_t *buffer, size_t capacity, Sx127xLoRaPacket &packet)
    {
        if (!initialized || rxFrames.empty())
        {
            return false;
        }
        const std::vector<uint8_t> frame = rxFrames.front();
        rxFrames.pop_front();
        packet.length = frame.size();
        if (frame.size() > capacity)
        {
            return false;
        }
        for (size_t i = 0U; i < frame.size(); ++i)
        {
            buffer[i] = frame[i];
        }
        return true;
    }

    bool send(const uint8_t *data, size_t length, bool async = false)
    {
        (void)async;
        if (!initialized || data == nullptr || length == 0U)
        {
            return false;
        }
        txFrames.emplace_back(data, data + length);
        return true;
    }

    void queueRx(const std::vector<uint8_t> &frame)
    {
        rxFrames.push_back(frame);
    }

    bool beginResult = true;
    bool initialized = false;
    Sx127xLoRaConfig lastConfig;
    std::deque<std::vector<uint8_t>> rxFrames;
    std::vector<std::vector<uint8_t>> txFrames;
};
