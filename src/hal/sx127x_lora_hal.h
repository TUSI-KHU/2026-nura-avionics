#pragma once

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

struct Sx127xLoRaConfig
{
    // RFM95W/RFM96W and RA-01 share the Semtech SX127x register-level family.
    long frequencyHz = 915000000L;
    int ssPin = 10;
    int resetPin = 9;
    int dio0Pin = 2;
    uint32_t spiFrequency = 8000000UL;
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

    bool initialized_ = false;
};
