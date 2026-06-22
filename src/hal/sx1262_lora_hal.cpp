#include "sx1262_lora_hal.h"

#include <math.h>

namespace
{
bool succeeded(int16_t state)
{
    return state == RADIOLIB_ERR_NONE;
}
} // namespace

bool Sx1262LoRaHAL::begin(const Sx1262LoRaConfig &config)
{
    initialized_ = false;

    if (!applyConfig(config))
    {
        return false;
    }

    // The PCB does not record an MCU-controlled SX1262 NRESET net. RadioLib's
    // no-reset mode is intentional here; D30/RXE is not driven until its RF
    // front-end role and polarity are verified from the schematic.
    const int16_t state = radio_.begin(static_cast<float>(config.frequencyHz) / 1000000.0f,
                                       static_cast<float>(config.signalBandwidthHz) / 1000.0f,
                                       static_cast<uint8_t>(config.spreadingFactor),
                                       static_cast<uint8_t>(config.codingRateDenominator),
                                       static_cast<uint8_t>(config.syncWord),
                                       static_cast<int8_t>(config.txPowerDbm),
                                       static_cast<uint16_t>(config.preambleLength),
                                       config.tcxoVoltage,
                                       config.useRegulatorLdo);
    if (!succeeded(state))
    {
        return false;
    }

    if (!succeeded(radio_.explicitHeader()) ||
        !succeeded(radio_.setCRC(config.crcEnabled ? 2U : 0U)))
    {
        radio_.sleep();
        return false;
    }
#if !defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    if (!startReceive())
    {
        radio_.sleep();
        return false;
    }
#endif

    initialized_ = true;
    return true;
}

void Sx1262LoRaHAL::end()
{
    if (initialized_)
    {
        radio_.sleep();
    }
    initialized_ = false;
}

bool Sx1262LoRaHAL::send(const uint8_t *data, size_t length)
{
    if (!initialized_ || data == nullptr || length == 0U)
    {
        return false;
    }

    const int16_t transmitState = radio_.transmit(data, length);
    int16_t receiveState = RADIOLIB_ERR_NONE;
#if !defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    receiveState = radio_.startReceive();
#endif
#if defined(NURA_BENCH_SX1262_RXE_LOW)
    if (!succeeded(transmitState) || !succeeded(receiveState))
    {
        Serial.print("SX1262_TX_STATE=");
        Serial.print(transmitState);
        Serial.print(" RX_STATE=");
        Serial.println(receiveState);
    }
#endif
    const bool transmitted = succeeded(transmitState);
    const bool receiving = succeeded(receiveState);
    return transmitted && receiving;
}

bool Sx1262LoRaHAL::receive(uint8_t *buffer, size_t capacity, Sx1262LoRaPacket &packet)
{
    packet = Sx1262LoRaPacket{};
#if defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    (void)buffer;
    (void)capacity;
    return false;
#else
    if (!initialized_ || buffer == nullptr || capacity == 0U)
    {
        return false;
    }

    if ((radio_.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) == 0U)
    {
        return false;
    }

    const size_t receivedLength = radio_.getPacketLength();
    packet.rssi = static_cast<int>(lroundf(radio_.getRSSI()));
    packet.snr = radio_.getSNR();
    packet.frequencyError = static_cast<long>(lroundf(radio_.getFrequencyError()));

    const bool overflow = receivedLength > capacity;
    const int16_t state = radio_.readData(buffer, capacity);
    const bool receiving = startReceive();
    if (!succeeded(state) || !receiving || overflow)
    {
        return false;
    }

    packet.length = receivedLength;
    return true;
#endif
}

int Sx1262LoRaHAL::rssi()
{
    if (!initialized_)
    {
        return 0;
    }
    return static_cast<int>(lroundf(radio_.getRSSI(false)));
}

bool Sx1262LoRaHAL::applyConfig(const Sx1262LoRaConfig &config)
{
    return config.frequencyHz >= 150000000L &&
           config.frequencyHz <= 960000000L &&
           config.txPowerDbm >= -9 &&
           config.txPowerDbm <= 22 &&
           config.spreadingFactor >= 5 &&
           config.spreadingFactor <= 12 &&
           config.signalBandwidthHz > 0L &&
           config.codingRateDenominator >= 5 &&
           config.codingRateDenominator <= 8 &&
           config.preambleLength >= 1L &&
           config.preambleLength <= 65535L &&
           config.syncWord >= 0 &&
           config.syncWord <= 255;
}

bool Sx1262LoRaHAL::startReceive()
{
    return succeeded(radio_.startReceive());
}
