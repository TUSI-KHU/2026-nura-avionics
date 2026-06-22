#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "nura_constants.h"
#include "nura_protocol_v1_lite.h"

namespace
{
constexpr uint8_t kMisoPin = 1U;
constexpr uint8_t kMosiPin = 26U;
constexpr uint8_t kSckPin = 27U;
constexpr uint8_t kNssPin = 9U;
constexpr uint8_t kRxEnablePin = 30U;
constexpr uint8_t kDio1Pin = 31U;
constexpr uint8_t kBusyPin = 32U;

constexpr float kFrequencyMHz = 920.9F;
constexpr float kBandwidthKHz = 125.0F;
constexpr uint8_t kSpreadingFactor = 7U;
constexpr uint8_t kCodingRate = 5U;
constexpr uint8_t kSyncWord = 0x12U;
constexpr int8_t kTxPowerDbm = 2;
constexpr uint16_t kPreambleLength = 8U;
constexpr uint32_t kTxIntervalMs = 1000UL;

#if defined(NURA_BENCH_ENABLE_TX)
constexpr bool kTransmitEnabled = true;
#else
constexpr bool kTransmitEnabled = false;
#endif

Module radioModule(kNssPin,
                   kDio1Pin,
                   RADIOLIB_NC,
                   kBusyPin,
                   SPI1,
                   SPISettings(2000000UL, MSBFIRST, SPI_MODE0));
SX1262 radio(&radioModule);

bool radioReady = false;
int16_t radioInitState = RADIOLIB_ERR_UNKNOWN;
uint16_t frameSequence = 0U;
uint32_t lastTxMs = 0UL;
uint32_t lastStatusMs = 0UL;

uint8_t readRawStatus()
{
    SPI1.beginTransaction(SPISettings(125000UL, MSBFIRST, SPI_MODE0));
    digitalWrite(kNssPin, LOW);
    delayMicroseconds(20);
    SPI1.transfer(0xC0U);
    const uint8_t status = SPI1.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(kNssPin, HIGH);
    SPI1.endTransaction();
    return status;
}

size_t buildFastFrame(uint8_t *frame, size_t capacity)
{
    nura::FastTelemetry fast;
    fast.statusWord = nura::statusWithFlightState(nura::STATUS_RADIO_OK,
                                                  nura::FLIGHT_BOOT);
    fast.bootMs = millis();
    fast.battMv = 0U;

    uint8_t payload[nura::kFastPayloadLen];
    if (!nura::encodeFastPayload(fast, payload, sizeof(payload)))
    {
        return 0U;
    }

    return nura::encodeFrame(nura::MESSAGE_FAST_TLM,
                             NuraConstants::Telemetry::kVehicleId,
                             frameSequence,
                             nura::FrameDirection::DOWNLINK,
                             NuraConstants::Telemetry::kControlAuthKey,
                             payload,
                             sizeof(payload),
                             frame,
                             capacity);
}
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000UL)
    {
    }

    SPI1.setMISO(kMisoPin);
    SPI1.setMOSI(kMosiPin);
    SPI1.setSCK(kSckPin);
    SPI1.begin();
    pinMode(kNssPin, OUTPUT);
    digitalWrite(kNssPin, HIGH);
    pinMode(kRxEnablePin, OUTPUT);
    digitalWrite(kRxEnablePin, LOW);

    Serial.println("AVIONICS_SX1262_BENCH");
    Serial.print("mode=");
    Serial.println(kTransmitEnabled ? "tx_2dbm" : "init_only_no_tx");

    radioInitState = radio.begin(kFrequencyMHz,
                                 kBandwidthKHz,
                                 kSpreadingFactor,
                                 kCodingRate,
                                 kSyncWord,
                                 kTxPowerDbm,
                                 kPreambleLength,
                                 0.0F,
                                 false);
    if (radioInitState != RADIOLIB_ERR_NONE)
    {
        Serial.print("SX1262_INIT_FAIL code=");
        Serial.println(radioInitState);
        return;
    }

    radioReady = true;
    Serial.println("SX1262_INIT_OK");
}

void loop()
{
    const uint32_t nowMs = millis();
    if ((nowMs - lastStatusMs) >= 1000UL)
    {
        lastStatusMs = nowMs;
        Serial.print("status radio=");
        Serial.print(radioReady ? "ready" : "not_ready");
        Serial.print(" init_code=");
        Serial.print(radioInitState);
        Serial.print(" rxe=");
        Serial.print(digitalRead(kRxEnablePin));
        Serial.print(" dio1=");
        Serial.print(digitalRead(kDio1Pin));
        Serial.print(" busy=");
        Serial.print(digitalRead(kBusyPin));
        Serial.print(" raw_status=0x");
        const uint8_t rawStatus = readRawStatus();
        if (rawStatus < 16U)
        {
            Serial.print('0');
        }
        Serial.print(rawStatus, HEX);
        Serial.print(" tx=");
        Serial.println(kTransmitEnabled ? "enabled" : "disabled");
    }

    if (!radioReady || !kTransmitEnabled ||
        (nowMs - lastTxMs) < kTxIntervalMs)
    {
        return;
    }
    lastTxMs = nowMs;

    uint8_t frame[nura::kMaxFrameLen];
    const size_t frameLength = buildFastFrame(frame, sizeof(frame));
    if (frameLength == 0U)
    {
        Serial.println("TX_ENCODE_FAIL");
        return;
    }

    const int16_t state = radio.transmit(frame, frameLength);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.print("TX_OK seq=");
        Serial.print(frameSequence);
        Serial.print(" len=");
        Serial.println(frameLength);
        ++frameSequence;
    }
    else
    {
        Serial.print("TX_FAIL code=");
        Serial.println(state);
    }
}
