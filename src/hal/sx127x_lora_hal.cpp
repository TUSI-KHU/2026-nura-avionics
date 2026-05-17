#include "sx127x_lora_hal.h"

#define private public
#include <LoRa.h>
#undef private

namespace
{
    constexpr uint8_t kRegVersion = 0x42U;
    constexpr uint8_t kExpectedVersion = 0x12U;
}

bool Sx127xLoRaHAL::begin(const Sx127xLoRaConfig &config, SPIClass &spi)
{
    initialized_ = false;
    selectedSpiFrequency_ = config.spiFrequency;

    LoRa.setPins(config.ssPin, config.libraryResetPin, config.dio0Pin);
    LoRa.setSPI(spi);
    LoRa.setSPIFrequency(config.spiFrequency);

    const uint8_t attempts = config.initAttempts == 0U ? 1U : config.initAttempts;
    for (uint8_t attempt = 0U; attempt < attempts; ++attempt)
    {
        resetRadio(config);
        if (!selectSpiMode(config, spi))
        {
            delay(250);
            continue;
        }

        LoRa._spiSettings = SPISettings(config.spiFrequency, MSBFIRST, selectedSpiMode_);
        if (LoRa.begin(config.frequencyHz))
        {
            LoRa._spiSettings = SPISettings(config.spiFrequency, MSBFIRST, selectedSpiMode_);
            if (!applyConfig(config))
            {
                LoRa.end();
                return false;
            }

            LoRa.receive();
            initialized_ = true;
            return true;
        }

        LoRa.end();
        delay(250);
    }

    return false;
}

void Sx127xLoRaHAL::end()
{
    if (initialized_)
    {
        LoRa.end();
    }
    initialized_ = false;
}

bool Sx127xLoRaHAL::send(const uint8_t *data, size_t length, bool async)
{
    if (!initialized_ || data == nullptr || length == 0U)
    {
        return false;
    }

    LoRa._spiSettings = SPISettings(selectedSpiFrequency_, MSBFIRST, selectedSpiMode_);
    LoRa.idle();
    delay(2);
    if (!LoRa.beginPacket())
    {
        LoRa.receive();
        return false;
    }

    const size_t written = LoRa.write(data, length);
    if (written != length)
    {
        LoRa.receive();
        return false;
    }

    const bool ok = LoRa.endPacket(async) == 1;
    if (!async)
    {
        LoRa.receive();
    }
    return ok;
}

bool Sx127xLoRaHAL::receive(uint8_t *buffer, size_t capacity, Sx127xLoRaPacket &packet)
{
    if (!initialized_ || buffer == nullptr || capacity == 0U)
    {
        return false;
    }

    const int packetSize = LoRa.parsePacket();
    if (packetSize <= 0)
    {
        packet.length = 0U;
        return false;
    }

    size_t count = 0U;
    bool overflow = false;
    while (LoRa.available())
    {
        const int value = LoRa.read();
        if (value < 0)
        {
            break;
        }
        if (count < capacity)
        {
            buffer[count] = static_cast<uint8_t>(value);
            ++count;
        }
        else
        {
            overflow = true;
        }
    }

    packet.length = count;
    packet.rssi = LoRa.packetRssi();
    packet.snr = LoRa.packetSnr();
    packet.frequencyError = LoRa.packetFrequencyError();

    return !overflow && count == static_cast<size_t>(packetSize);
}

int Sx127xLoRaHAL::rssi() const
{
    if (!initialized_)
    {
        return 0;
    }
    return LoRa.rssi();
}

bool Sx127xLoRaHAL::applyConfig(const Sx127xLoRaConfig &config)
{
    if (config.spreadingFactor < 6 || config.spreadingFactor > 12)
    {
        return false;
    }

    if (config.codingRateDenominator < 5 || config.codingRateDenominator > 8)
    {
        return false;
    }

    LoRa.setTxPower(config.txPowerDbm);
    LoRa.setSpreadingFactor(config.spreadingFactor);
    LoRa.setSignalBandwidth(config.signalBandwidthHz);
    LoRa.setCodingRate4(config.codingRateDenominator);
    LoRa.setPreambleLength(config.preambleLength);
    LoRa.setSyncWord(config.syncWord);

    if (config.crcEnabled)
    {
        LoRa.enableCrc();
    }
    else
    {
        LoRa.disableCrc();
    }

    return true;
}

bool Sx127xLoRaHAL::selectSpiMode(const Sx127xLoRaConfig &config, SPIClass &spi)
{
    if (!config.probeSpiMode)
    {
        selectedSpiMode_ = config.spiMode;
        return true;
    }

    const uint8_t modes[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (readRegisterRaw(config, spi, kRegVersion, modes[i]) == kExpectedVersion)
        {
            selectedSpiMode_ = modes[i];
            return true;
        }
    }

    return false;
}

uint8_t Sx127xLoRaHAL::readRegisterRaw(const Sx127xLoRaConfig &config, SPIClass &spi, uint8_t address, uint8_t spiMode)
{
    SPISettings settings(config.spiFrequency, MSBFIRST, spiMode);
    pinMode(config.ssPin, OUTPUT);
    digitalWrite(config.ssPin, HIGH);
    spi.beginTransaction(settings);
    digitalWrite(config.ssPin, LOW);
    delayMicroseconds(20);
    spi.transfer(address & 0x7FU);
    const uint8_t value = spi.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(config.ssPin, HIGH);
    spi.endTransaction();
    return value;
}

void Sx127xLoRaHAL::resetRadio(const Sx127xLoRaConfig &config)
{
    if (config.resetPin < 0)
    {
        return;
    }

    pinMode(config.ssPin, OUTPUT);
    digitalWrite(config.ssPin, HIGH);
    pinMode(config.resetPin, OUTPUT);
    digitalWrite(config.resetPin, LOW);
    delay(50);
    digitalWrite(config.resetPin, HIGH);
    delay(500);
}
