#include "sx1262_lora_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
bool succeeded(int16_t state)
{
    return state == RADIOLIB_ERR_NONE;
}

void readVersionString(uint8_t (&version)[16])
{
    SPI1.beginTransaction(SPISettings(NuraConstants::LoRa::kFlightSpiFrequencyHz, MSBFIRST, SPI_MODE0));
    digitalWrite(BoardPinMap::Sx1262LoRa::ssPin, LOW);
    (void)SPI1.transfer(0x1D);
    (void)SPI1.transfer(0x03);
    (void)SPI1.transfer(0x20);
    (void)SPI1.transfer(0x00);
    for (uint8_t i = 0; i < sizeof(version); ++i)
    {
        version[i] = SPI1.transfer(0x00);
    }
    digitalWrite(BoardPinMap::Sx1262LoRa::ssPin, HIGH);
    SPI1.endTransaction();
}

#if defined(NURA_BENCH_RADIO_TRACE)
void traceRadio(const char *label)
{
    if (!Serial)
    {
        return;
    }

    Serial.print("SX1262_");
    Serial.print(label);
    Serial.print(" ms=");
    Serial.println(millis());
}

void traceRadioState(const char *label, int16_t state)
{
    if (!Serial)
    {
        return;
    }

    Serial.print("SX1262_");
    Serial.print(label);
    Serial.print(" state=");
    Serial.print(state);
    Serial.print(" ms=");
    Serial.println(millis());
}

void traceRadioFlags(const char *label, uint32_t flags)
{
    if (!Serial)
    {
        return;
    }

    Serial.print("SX1262_");
    Serial.print(label);
    Serial.print(" flags=0x");
    Serial.print(flags, HEX);
    Serial.print(" ms=");
    Serial.println(millis());
}

void tracePinLevels()
{
    if (!Serial)
    {
        return;
    }

    Serial.print("SX1262_PINS nss=");
    Serial.print(digitalRead(BoardPinMap::Sx1262LoRa::ssPin));
    Serial.print(" dio1=");
    Serial.print(digitalRead(BoardPinMap::Sx1262LoRa::dio1Pin));
    Serial.print(" busy=");
    Serial.print(digitalRead(BoardPinMap::Sx1262LoRa::busyPin));
    Serial.print(" rxe=");
    Serial.print(digitalRead(BoardPinMap::Sx1262LoRa::rxEnablePin));
    Serial.print(" ms=");
    Serial.println(millis());
}

void traceVersionProbe()
{
    if (!Serial)
    {
        return;
    }

    uint8_t version[16] = {};
    readVersionString(version);

    Serial.print("SX1262_VERSION_RAW");
    for (uint8_t i = 0; i < sizeof(version); ++i)
    {
        Serial.print(i == 0U ? " " : ":");
        if (version[i] < 0x10U)
        {
            Serial.print('0');
        }
        Serial.print(version[i], HEX);
    }
    Serial.print(" ms=");
    Serial.println(millis());
}
#else
#define traceRadio(label) ((void)0)
#define traceRadioState(label, state) ((void)0)
#define traceRadioFlags(label, flags) ((void)0)
#define tracePinLevels() ((void)0)
#define traceVersionProbe() ((void)0)
#endif
} // namespace

bool Sx1262LoRaHAL::begin(const Sx1262LoRaConfig &config)
{
    initialized_ = false;
    txBusy_ = false;

    if (!applyConfig(config))
    {
        return false;
    }
    config_ = config;
    configValid_ = true;
    setReceivePath(false);

    // The PCB does not record an MCU-controlled SX1262 NRESET net. RadioLib's
    // no-reset mode is intentional here. The short version read wakes/probes
    // the radio on this PCB before RadioLib performs its own chip check.
    tracePinLevels();
#if defined(NURA_BENCH_RADIO_TRACE)
    traceVersionProbe();
#else
    uint8_t ignoredVersion[16] = {};
    readVersionString(ignoredVersion);
#endif
    traceRadio("BEGIN_CALL");
    const int16_t state = radio_.begin(static_cast<float>(config.frequencyHz) / 1000000.0f,
                                       static_cast<float>(config.signalBandwidthHz) / 1000.0f,
                                       static_cast<uint8_t>(config.spreadingFactor),
                                       static_cast<uint8_t>(config.codingRateDenominator),
                                       static_cast<uint8_t>(config.syncWord),
                                       static_cast<int8_t>(config.txPowerDbm),
                                       static_cast<uint16_t>(config.preambleLength),
                                       config.tcxoVoltage,
                                       config.useRegulatorLdo);
    traceRadioState("BEGIN_RET", state);
    if (!succeeded(state))
    {
        return false;
    }

#if !defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    traceRadio("START_RX_INIT_CALL");
    if (!startReceive())
    {
        radio_.sleep();
        return false;
    }
    traceRadio("START_RX_INIT_RET");
#endif

    initialized_ = true;
    return true;
}

void Sx1262LoRaHAL::end()
{
    txBusy_ = false;
    if (initialized_)
    {
        radio_.sleep();
    }
    initialized_ = false;
}

void Sx1262LoRaHAL::service(uint32_t nowMs)
{
    if (!initialized_ || !txBusy_)
    {
        return;
    }

    if (digitalRead(BoardPinMap::Sx1262LoRa::dio1Pin) == HIGH)
    {
        (void)finishTransmitAndRestartReceive();
        return;
    }

    if (txTimeoutMs_ != 0UL && (nowMs - txStartMs_) > txTimeoutMs_)
    {
        traceRadio("TX_TIMEOUT");
        abortTransmit();
    }
}

bool Sx1262LoRaHAL::txBusy() const
{
    return txBusy_;
}

bool Sx1262LoRaHAL::send(const uint8_t *data, size_t length)
{
    if (!initialized_ || txBusy_ || data == nullptr || length == 0U)
    {
#if defined(NURA_BENCH_RADIO_REINIT_ON_TX_FAIL)
        if (!initialized_ && data != nullptr && length > 0U && configValid_ && begin(config_))
        {
            return send(data, length);
        }
#endif
        return false;
    }

    traceRadio("TX_CALL");
    setReceivePath(false);
    int16_t transmitState = radio_.startTransmit(data, length);
    traceRadioState("TX_RET", transmitState);

    if (!succeeded(transmitState) && configValid_)
    {
        initialized_ = false;
        traceRadio("REINIT_AFTER_TX_FAIL_CALL");
        if (begin(config_))
        {
            traceRadio("RETX_CALL");
            setReceivePath(false);
            transmitState = radio_.startTransmit(data, length);
            traceRadioState("RETX_RET", transmitState);
        }
    }
#if defined(NURA_BENCH_SX1262_RXE_LOW)
    if (!succeeded(transmitState))
    {
        Serial.print("SX1262_TX_STATE=");
        Serial.print(transmitState);
        Serial.println(" RX_STATE=busy");
    }
#endif
    if (!succeeded(transmitState))
    {
        setReceivePath(false);
        return false;
    }

    const RadioLibTime_t timeOnAirUs = radio_.getTimeOnAir(length);
    txStartMs_ = millis();
    txTimeoutMs_ = 10UL + static_cast<uint32_t>((timeOnAirUs * 5UL) / 1000UL);
    txBusy_ = true;
    return true;
}

bool Sx1262LoRaHAL::receive(uint8_t *buffer, size_t capacity, Sx1262LoRaPacket &packet)
{
    packet = Sx1262LoRaPacket{};
#if defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    (void)buffer;
    (void)capacity;
    return false;
#else
    if (!initialized_ || txBusy_ || buffer == nullptr || capacity == 0U)
    {
        return false;
    }

    if (digitalRead(BoardPinMap::Sx1262LoRa::dio1Pin) == LOW)
    {
        return false;
    }

    traceRadio("RX_IRQ_CALL");
    const uint32_t irqFlags = radio_.getIrqFlags();
    traceRadioFlags("RX_IRQ_RET", irqFlags);
    if ((irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE) == 0U)
    {
        return false;
    }

    const size_t receivedLength = radio_.getPacketLength();
    packet.rssi = static_cast<int>(lroundf(radio_.getRSSI()));
    packet.snr = radio_.getSNR();
    packet.frequencyError = static_cast<long>(lroundf(radio_.getFrequencyError()));

    const bool overflow = receivedLength > capacity;
    traceRadio("RX_READ_CALL");
    const int16_t state = radio_.readData(buffer, capacity);
    traceRadioState("RX_READ_RET", state);
    traceRadio("START_RX_AFTER_READ_CALL");
    const bool receiving = startReceive();
    traceRadio(receiving ? "START_RX_AFTER_READ_RET_OK" : "START_RX_AFTER_READ_RET_FAIL");
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

void Sx1262LoRaHAL::setReceivePath(bool enabled)
{
    pinMode(BoardPinMap::Sx1262LoRa::rxEnablePin, OUTPUT);
    digitalWrite(BoardPinMap::Sx1262LoRa::rxEnablePin, enabled ? HIGH : LOW);
}

bool Sx1262LoRaHAL::startReceive()
{
    const int16_t state = radio_.startReceive();
    traceRadioState("START_RX_RET", state);
    return succeeded(state);
}

bool Sx1262LoRaHAL::finishTransmitAndRestartReceive()
{
    traceRadio("FINISH_TX_CALL");
    const int16_t finishState = radio_.finishTransmit();
    traceRadioState("FINISH_TX_RET", finishState);
    txBusy_ = false;

#if !defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    traceRadio("START_RX_AFTER_TX_CALL");
    const int16_t receiveState = radio_.startReceive();
    traceRadioState("START_RX_AFTER_TX_RET", receiveState);
    if (succeeded(finishState) && succeeded(receiveState))
    {
        return true;
    }

    if (configValid_)
    {
        initialized_ = false;
        traceRadio("REINIT_AFTER_TX_FINISH_FAIL_CALL");
        initialized_ = begin(config_);
        traceRadio(initialized_ ? "REINIT_AFTER_TX_FINISH_FAIL_OK" : "REINIT_AFTER_TX_FINISH_FAIL_BAD");
    }
    return false;
#else
    setReceivePath(false);
    if (succeeded(finishState))
    {
        return true;
    }
    if (configValid_)
    {
        initialized_ = false;
        traceRadio("REINIT_AFTER_TX_FINISH_FAIL_CALL");
        initialized_ = begin(config_);
        traceRadio(initialized_ ? "REINIT_AFTER_TX_FINISH_FAIL_OK" : "REINIT_AFTER_TX_FINISH_FAIL_BAD");
    }
    return false;
#endif
}

void Sx1262LoRaHAL::abortTransmit()
{
    traceRadio("ABORT_TX_CALL");
    (void)radio_.finishTransmit();
    txBusy_ = false;
#if !defined(NURA_BENCH_RADIO_DOWNLINK_ONLY)
    (void)startReceive();
#endif
}
