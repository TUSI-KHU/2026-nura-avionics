#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "board_pinmap.h"
#include "hal/sx127x_lora_hal.h"
#include "nura_constants.h"
#include "nura_protocol_v1_lite.h"

namespace
{
constexpr uint32_t kTxPeriodMs = 750UL;
constexpr uint32_t kInitRetryPeriodMs = 1000UL;
constexpr uint16_t kTxBlinkMs = 80U;
constexpr uint16_t kFailBlinkMs = 300U;

Sx127xLoRaHAL radio;
uint16_t downlinkSeq = 0U;
uint32_t lastTxMs = 0UL;
uint32_t lastInitRetryMs = 0UL;
uint32_t txCount = 0UL;
uint32_t failCount = 0UL;
bool radioReady = false;

Sx127xLoRaConfig buildRadioConfig();
nura::FastTelemetry buildFastTelemetry(uint32_t nowMs);
void emitFastLine(uint16_t seq, const nura::FastTelemetry &fast, uint32_t nowMs);
const char *flightStateName(uint8_t state);
int16_t clampToI16(float value);
float flightAltitudeM(float phaseS);
uint8_t profileFlightState(float phaseS);

bool serialConsoleOpen()
{
    return true;
}

void setLed(uint8_t pin, bool enabled)
{
    if (pin == BoardPinMap::kUnassignedPin)
    {
        return;
    }

    digitalWrite(pin, enabled ? HIGH : LOW);
}

void pulseLed(uint8_t pin, uint16_t durationMs)
{
    setLed(pin, true);
    delay(durationMs);
    setLed(pin, false);
}

bool sendFastFrame(uint32_t nowMs)
{
    nura::FastTelemetry fast = buildFastTelemetry(nowMs);
    uint8_t payload[nura::kFastPayloadLen];
    uint8_t frame[nura::kMaxFrameLen];
    if (!nura::encodeFastPayload(fast, payload, sizeof(payload)))
    {
        return false;
    }

    const uint16_t seq = downlinkSeq++;
    emitFastLine(seq, fast, nowMs);
    const size_t frameLen = nura::encodeFrame(nura::MESSAGE_FAST_TLM,
                                              NuraConstants::Telemetry::kVehicleId,
                                              seq,
                                              nura::FrameDirection::DOWNLINK,
                                              NuraConstants::Telemetry::kControlAuthKey,
                                              payload,
                                              nura::kFastPayloadLen,
                                              frame,
                                              sizeof(frame));
    if (frameLen == 0U)
    {
        return false;
    }

    const bool ok = radio.send(frame, frameLen);
    return ok;
}

nura::FastTelemetry buildFastTelemetry(uint32_t nowMs)
{
    const float t = nowMs / 1000.0F;
    const float phaseS = fmodf(t, 45.0F);
    const float altitudeM = flightAltitudeM(phaseS);
    const float boost = phaseS < 3.0F ? (1.0F - (phaseS / 3.0F)) : 0.0F;
    const float descent = phaseS > 14.0F && phaseS < 39.0F ? 1.0F : 0.0F;
    const float axG = 0.10F * sinf(t * 0.71F) + 0.18F * boost;
    const float ayG = 0.07F * cosf(t * 0.43F);
    const float azG = 1.0F + 0.70F * boost - 0.22F * descent + 0.04F * sinf(t * 1.27F);
    const float gxDps = 5.0F * sinf(t * 0.91F);
    const float gyDps = 4.0F * cosf(t * 0.67F);
    const float gzDps = 10.0F * sinf(t * 0.31F) + 2.0F * cosf(t * 1.13F);
    const uint8_t state = profileFlightState(phaseS);

    nura::FastTelemetry fast;
    fast.statusWord = nura::statusWithFlightState(radioReady ? nura::STATUS_RADIO_OK : 0U, state);
    fast.bootMs = nowMs;
    fast.baroDp2Pa = clampToI16(-altitudeM * 6.0F);
    fast.lowAccelXCg = clampToI16(axG * 100.0F);
    fast.lowAccelYCg = clampToI16(ayG * 100.0F);
    fast.lowAccelZCg = clampToI16(azG * 100.0F);
    fast.gyroXDps10 = clampToI16(gxDps * 10.0F);
    fast.gyroYDps10 = clampToI16(gyDps * 10.0F);
    fast.gyroZDps10 = clampToI16(gzDps * 10.0F);
    fast.battMv = static_cast<uint16_t>(11800U + static_cast<uint16_t>(60.0F * sinf(t * 0.11F)));
    return fast;
}

int16_t clampToI16(float value)
{
    if (value > 32767.0F)
    {
        return 32767;
    }
    if (value < -32768.0F)
    {
        return -32768;
    }
    return static_cast<int16_t>(roundf(value));
}

float flightAltitudeM(float phaseS)
{
    if (phaseS < 3.0F)
    {
        return 18.0F * phaseS * phaseS;
    }
    if (phaseS < 11.0F)
    {
        const float x = phaseS - 3.0F;
        return 162.0F + (24.0F * x) - (0.9F * x * x);
    }
    if (phaseS < 14.0F)
    {
        return 296.0F - 2.0F * (phaseS - 11.0F);
    }
    if (phaseS < 39.0F)
    {
        const float x = phaseS - 14.0F;
        return max(0.0F, 290.0F - 11.5F * x);
    }
    return 0.0F;
}

uint8_t profileFlightState(float phaseS)
{
    if (phaseS < 1.0F)
    {
        return nura::FLIGHT_SAFE;
    }
    if (phaseS < 2.0F)
    {
        return nura::FLIGHT_ARMED;
    }
    if (phaseS < 4.0F)
    {
        return nura::FLIGHT_LAUNCH;
    }
    if (phaseS < 11.0F)
    {
        return nura::FLIGHT_COAST;
    }
    if (phaseS < 14.0F)
    {
        return nura::FLIGHT_APOGEE;
    }
    if (phaseS < 31.0F)
    {
        return nura::FLIGHT_DROGUE;
    }
    if (phaseS < 39.0F)
    {
        return nura::FLIGHT_DEPLOY;
    }
    return nura::FLIGHT_GROUND;
}

const char *flightStateName(uint8_t state)
{
    switch (state)
    {
    case nura::FLIGHT_SAFE:
        return "SAFE";
    case nura::FLIGHT_ARMED:
        return "ARMED";
    case nura::FLIGHT_LAUNCH:
        return "LAUNCH";
    case nura::FLIGHT_COAST:
        return "COAST";
    case nura::FLIGHT_APOGEE:
        return "APOGEE";
    case nura::FLIGHT_DROGUE:
        return "DROGUE";
    case nura::FLIGHT_DEPLOY:
        return "DEPLOY";
    case nura::FLIGHT_GROUND:
        return "GROUND";
    default:
        return "INIT";
    }
}

void emitFastLine(uint16_t seq, const nura::FastTelemetry &fast, uint32_t nowMs)
{
    if (!serialConsoleOpen())
    {
        return;
    }
    Serial.print("rx type=FAST seq=");
    Serial.print(seq);
    Serial.print(" boot_ms=");
    Serial.print(nowMs);
    const uint8_t state = (fast.statusWord >> 8) & 0x0FU;
    Serial.print(" state=");
    Serial.print(flightStateName(state));
    Serial.print(" state_code=");
    Serial.print(state);
    Serial.print(" status=0x");
    if (fast.statusWord < 0x1000U)
    {
        Serial.print('0');
    }
    if (fast.statusWord < 0x0100U)
    {
        Serial.print('0');
    }
    if (fast.statusWord < 0x0010U)
    {
        Serial.print('0');
    }
    Serial.print(fast.statusWord, HEX);
    Serial.print(" baro_dp_2pa=");
    Serial.print(fast.baroDp2Pa);
    Serial.print(" accel_g=(");
    Serial.print(fast.lowAccelXCg / 100.0F, 2);
    Serial.print(",");
    Serial.print(fast.lowAccelYCg / 100.0F, 2);
    Serial.print(",");
    Serial.print(fast.lowAccelZCg / 100.0F, 2);
    Serial.print(") gyro_dps=(");
    Serial.print(fast.gyroXDps10 / 10.0F, 1);
    Serial.print(",");
    Serial.print(fast.gyroYDps10 / 10.0F, 1);
    Serial.print(",");
    Serial.print(fast.gyroZDps10 / 10.0F, 1);
    Serial.print(") batt_mv=");
    Serial.print(fast.battMv);
    Serial.println(radioReady ? " health=-,-,-,radio rssi=0 snr=0.00" : " health=-,-,-,usb rssi=0 snr=0.00");
}

bool beginRadioWithStatus()
{
    radioReady = radio.begin(buildRadioConfig(), SPI1);
    if (radioReady)
    {
        if (serialConsoleOpen())
        {
            Serial.println("PASS: sx1276 init OK");
        }
        pulseLed(BoardPinMap::StatusIndicator::led1Pin, kTxBlinkMs);
    }
    else
    {
        if (serialConsoleOpen())
        {
            Serial.println("FAIL: sx1276 init failed");
        }
        pulseLed(BoardPinMap::StatusIndicator::led2Pin, kFailBlinkMs);
    }
    return radioReady;
}

Sx127xLoRaConfig buildRadioConfig()
{
    Sx127xLoRaConfig config;
    config.ssPin = BoardPinMap::Sx1262LoRa::ssPin;
    config.resetPin = BoardPinMap::Sx1262LoRa::resetPin;
    config.libraryResetPin = BoardPinMap::Sx1262LoRa::resetPin;
    config.dio0Pin = BoardPinMap::Sx1262LoRa::dio1Pin;
    config.frequencyHz = NuraConstants::LoRa::kFlightFrequencyHz;
    config.spiFrequency = NuraConstants::LoRa::kFlightSpiFrequencyHz;
    config.spiMode = NuraConstants::LoRa::kFlightSpiMode;
    config.probeSpiMode = true;
    config.initAttempts = NuraConstants::LoRa::kFlightInitAttempts;
    config.txPowerDbm = 2;
    config.spreadingFactor = NuraConstants::LoRa::kSpreadingFactor;
    config.signalBandwidthHz = NuraConstants::LoRa::kSignalBandwidthHz;
    config.codingRateDenominator = NuraConstants::LoRa::kCodingRateDenominator;
    config.preambleLength = NuraConstants::LoRa::kPreambleLength;
    config.syncWord = NuraConstants::LoRa::kSyncWord;
    config.crcEnabled = true;
    config.downlinkOnly = true;
    return config;
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    delay(1000);

    pinMode(BoardPinMap::StatusIndicator::led1Pin, OUTPUT);
    pinMode(BoardPinMap::StatusIndicator::led2Pin, OUTPUT);
    setLed(BoardPinMap::StatusIndicator::led1Pin, false);
    setLed(BoardPinMap::StatusIndicator::led2Pin, false);

    if (BoardPinMap::Sx1262LoRa::rxEnablePin != BoardPinMap::kUnassignedPin)
    {
        pinMode(BoardPinMap::Sx1262LoRa::rxEnablePin, OUTPUT);
        digitalWrite(BoardPinMap::Sx1262LoRa::rxEnablePin, LOW);
    }

    SPI1.setMISO(BoardPinMap::Spi1Bus::misoPin);
    SPI1.setMOSI(BoardPinMap::Spi1Bus::mosiPin);
    SPI1.setSCK(BoardPinMap::Spi1Bus::sckPin);
    SPI1.begin();

    if (serialConsoleOpen())
    {
        Serial.println("NURA SX1276 visual TX bench");
        Serial.println("role=avionics_visual_tx packet=FAST freq=920900000 tx_power_dbm=2");
        Serial.println("led1=tx_ok_pulse led2=tx_fail_pulse");
    }

    beginRadioWithStatus();
}

void loop()
{
    const uint32_t nowMs = millis();
    if (!radioReady)
    {
        if ((nowMs - lastTxMs) >= kTxPeriodMs)
        {
            lastTxMs = nowMs;
            const uint16_t seq = downlinkSeq++;
            const nura::FastTelemetry fast = buildFastTelemetry(nowMs);
            emitFastLine(seq, fast, nowMs);
            ++txCount;
        }
        if ((nowMs - lastInitRetryMs) >= kInitRetryPeriodMs)
        {
            lastInitRetryMs = nowMs;
            if (serialConsoleOpen())
            {
                Serial.print("tx wait radio_init_retry count=");
                Serial.println(static_cast<unsigned long>(failCount));
            }
            ++failCount;
            beginRadioWithStatus();
        }
        return;
    }

    if ((nowMs - lastTxMs) < kTxPeriodMs)
    {
        return;
    }

    lastTxMs = nowMs;
    const bool ok = sendFastFrame(nowMs);
    if (ok)
    {
        ++txCount;
        if (serialConsoleOpen())
        {
            Serial.print("tx ok count=");
            Serial.print(txCount);
            Serial.print(" seq=");
            Serial.println(static_cast<uint16_t>(downlinkSeq - 1U));
        }
        pulseLed(BoardPinMap::StatusIndicator::led1Pin, kTxBlinkMs);
    }
    else
    {
        ++failCount;
        if (serialConsoleOpen())
        {
            Serial.print("tx fail count=");
            Serial.println(failCount);
        }
        pulseLed(BoardPinMap::StatusIndicator::led2Pin, kFailBlinkMs);
        radio.end();
        radioReady = false;
        lastInitRetryMs = nowMs;
    }
}
