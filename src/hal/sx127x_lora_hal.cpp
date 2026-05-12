#include "sx127x_lora_hal.h"

bool Sx127xLoRaHAL::begin(const Sx127xLoRaConfig &config, SPIClass &spi)
{
    initialized_ = false;

    LoRa.setPins(config.ssPin, config.resetPin, config.dio0Pin);
    LoRa.setSPI(spi);
    LoRa.setSPIFrequency(config.spiFrequency);

    if (!LoRa.begin(config.frequencyHz))
    {
        return false;
    }

    if (!applyConfig(config))
    {
        LoRa.end();
        return false;
    }

    initialized_ = true;
    return true;
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

    if (!LoRa.beginPacket())
    {
        return false;
    }

    const size_t written = LoRa.write(data, length);
    if (written != length)
    {
        return false;
    }

    return LoRa.endPacket(async) == 1;
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
    while (LoRa.available() && count < capacity)
    {
        const int value = LoRa.read();
        if (value < 0)
        {
            break;
        }
        buffer[count] = static_cast<uint8_t>(value);
        ++count;
    }

    packet.length = count;
    packet.rssi = LoRa.packetRssi();
    packet.snr = LoRa.packetSnr();
    packet.frequencyError = LoRa.packetFrequencyError();

    return count == static_cast<size_t>(packetSize);
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
