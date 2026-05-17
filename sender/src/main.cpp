#include <Arduino.h>
#include <SPI.h>

#include "board_pinmap.h"
#include "nura_protocol_v1_lite.h"

#define private public
#include <LoRa.h>
#undef private

namespace
{
constexpr unsigned long kSerialBaud = 115200UL;
constexpr long kLoraFrequencyHz = 433000000L;
constexpr uint32_t kLoraSpiFrequencyHz = 125000UL;
constexpr int kLoraTxPowerDbm = 10;
constexpr int kLoraSpreadingFactor = 7;
constexpr long kLoraSignalBandwidthHz = 125000L;
constexpr int kLoraCodingRateDenominator = 5;
constexpr int kLoraSyncWord = 0x12;
constexpr uint8_t kLoraRegVersion = 0x42U;
constexpr uint8_t kLoraExpectedVersion = 0x12U;
constexpr uint8_t kLoraInitAttempts = 5U;

constexpr uint32_t kFastIntervalMs = 200UL;
constexpr uint32_t kGpsIntervalMs = 1000UL;

constexpr uint8_t kAuthKey[16] = {
    0x4e, 0x55, 0x52, 0x41, 0x2d, 0x56, 0x31, 0x4c,
    0x49, 0x54, 0x45, 0x2d, 0x54, 0x45, 0x53, 0x54};

uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0U;
bool radioReady = false;
uint16_t downlinkSeq = 0U;
uint32_t lastFastMs = 0UL;
uint32_t lastGpsMs = 0UL;
bool deployFired = false;
nura::Parser parser;

struct AckQueue
{
    nura::ControlPayload items[4];
    uint8_t head = 0U;
    uint8_t count = 0U;

    bool push(const nura::ControlPayload &item)
    {
        if (count >= 4U)
        {
            return false;
        }
        const uint8_t index = static_cast<uint8_t>((head + count) % 4U);
        items[index] = item;
        ++count;
        return true;
    }

    bool pop(nura::ControlPayload &out)
    {
        if (count == 0U)
        {
            return false;
        }
        out = items[head];
        head = static_cast<uint8_t>((head + 1U) % 4U);
        --count;
        return true;
    }
};

AckQueue ackQueue;

struct RecentCommand
{
    bool valid = false;
    uint8_t commandId = 0U;
    uint16_t commandSeq = 0U;
    uint32_t nonce = 0UL;
};

RecentCommand recentCommands[4];
uint8_t recentCommandWriteIndex = 0U;

void beginSpi()
{
#if defined(CORE_TEENSY)
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
#endif
    SPI.begin();
}

void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 16U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

uint8_t readLoraRegisterRaw(uint8_t address, uint8_t spiMode)
{
    SPISettings settings(kLoraSpiFrequencyHz, MSBFIRST, spiMode);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    SPI.beginTransaction(settings);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(address & 0x7FU);
    const uint8_t value = SPI.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    SPI.endTransaction();
    return value;
}

void resetRadio()
{
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::resetPin, OUTPUT);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::resetPin, LOW);
    delay(50);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::resetPin, HIGH);
    delay(500);
}

bool beginRadio()
{
    LoRa.setPins(BoardPinMap::Ra01DevelopmentLoRa::ssPin,
                 BoardPinMap::Ra01DevelopmentLoRa::libraryResetPin,
                 BoardPinMap::Ra01DevelopmentLoRa::dio0Pin);
    LoRa.setSPIFrequency(kLoraSpiFrequencyHz);

    for (uint8_t attempt = 1U; attempt <= kLoraInitAttempts; ++attempt)
    {
        beginSpi();
        resetRadio();

        const uint8_t mode0Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE0);
        const uint8_t mode1Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE1);
        const uint8_t mode2Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE2);
        const uint8_t mode3Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE3);

        Serial.print("init_attempt=");
        Serial.print(attempt);
        Serial.print(" m0=");
        printHexByte(mode0Version);
        Serial.print(" m1=");
        printHexByte(mode1Version);
        Serial.print(" m2=");
        printHexByte(mode2Version);
        Serial.print(" m3=");
        printHexByte(mode3Version);
        Serial.println();

        if (mode1Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE1;
            selectedSpiModeNumber = 1U;
        }
        else if (mode0Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else if (mode2Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE2;
            selectedSpiModeNumber = 2U;
        }
        else if (mode3Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE3;
            selectedSpiModeNumber = 3U;
        }
        else
        {
            delay(250);
            continue;
        }

        LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
        Serial.print("selected_spi_mode=");
        Serial.println(selectedSpiModeNumber);
        Serial.print("library_reg_version=");
        printHexByte(LoRa.readRegister(kLoraRegVersion));
        Serial.println();

        if (LoRa.begin(kLoraFrequencyHz))
        {
            LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
            LoRa.setTxPower(kLoraTxPowerDbm);
            LoRa.setSpreadingFactor(kLoraSpreadingFactor);
            LoRa.setSignalBandwidth(kLoraSignalBandwidthHz);
            LoRa.setCodingRate4(kLoraCodingRateDenominator);
            LoRa.setSyncWord(kLoraSyncWord);
            LoRa.enableCrc();
            LoRa.receive();
            return true;
        }

        LoRa.end();
        delay(250);
    }

    return false;
}

const char *commandName(uint8_t commandId)
{
    switch (commandId)
    {
    case nura::COMMAND_FORCE_DEPLOY_RECOVERY:
        return "FORCE_DEPLOY";
    case nura::COMMAND_ABORT_PROPULSION_DEPRECATED:
        return "ABORT_PROPULSION_DEPRECATED";
    case nura::COMMAND_SET_TELEMETRY_PROFILE:
        return "SET_TELEMETRY_PROFILE";
    default:
        return "UNKNOWN";
    }
}

const char *ackStageName(uint8_t stage)
{
    switch (stage)
    {
    case nura::ACK_RECEIVED:
        return "RECEIVED";
    case nura::ACK_ACCEPTED:
        return "ACCEPTED";
    case nura::ACK_EXECUTED:
        return "EXECUTED";
    case nura::ACK_REJECTED:
        return "REJECTED";
    case nura::ACK_DUPLICATE:
        return "DUPLICATE";
    default:
        return "UNKNOWN";
    }
}

const char *resultName(uint8_t result)
{
    switch (result)
    {
    case nura::RESULT_OK:
        return "OK";
    case nura::RESULT_AUTH_FAILED:
        return "AUTH_FAILED";
    case nura::RESULT_EXPIRED:
        return "EXPIRED";
    case nura::RESULT_NOT_SUPPORTED:
        return "NOT_SUPPORTED";
    case nura::RESULT_ALREADY_DONE:
        return "ALREADY_DONE";
    default:
        return "OTHER";
    }
}

uint8_t currentFlightState()
{
    return deployFired ? nura::FLIGHT_DESCENT : nura::FLIGHT_ARMED;
}

bool wasRecentlyProcessed(const nura::ControlPayload &command)
{
    for (const RecentCommand &recent : recentCommands)
    {
        if (recent.valid &&
            recent.commandId == command.commandId &&
            recent.commandSeq == command.commandSeq &&
            recent.nonce == command.nonce)
        {
            return true;
        }
    }
    return false;
}

void rememberCommand(const nura::ControlPayload &command)
{
    RecentCommand &slot = recentCommands[recentCommandWriteIndex];
    slot.valid = true;
    slot.commandId = command.commandId;
    slot.commandSeq = command.commandSeq;
    slot.nonce = command.nonce;
    recentCommandWriteIndex = static_cast<uint8_t>((recentCommandWriteIndex + 1U) % 4U);
}

void fillAck(nura::ControlPayload &ack,
             const nura::ControlPayload &command,
             uint8_t stage,
             uint8_t result,
             uint8_t reason,
             uint16_t detailFlags)
{
    ack = nura::ControlPayload{};
    ack.subtype = nura::CONTROL_ACK;
    ack.commandId = command.commandId;
    ack.commandSeq = command.commandSeq;
    ack.nonce = command.nonce;
    ack.validUntilMs = millis();
    ack.authOrAck[0] = stage;
    ack.authOrAck[1] = result;
    ack.authOrAck[2] = reason;
    ack.authOrAck[3] = currentFlightState();
    nura::writeU16(ack.authOrAck + 4, detailFlags);
    ack.authOrAck[6] = 0U;
    ack.authOrAck[7] = 0U;
}

void enqueueAck(const nura::ControlPayload &command,
                uint8_t stage,
                uint8_t result,
                uint8_t reason,
                uint16_t detailFlags = 0U)
{
    nura::ControlPayload ack;
    fillAck(ack, command, stage, result, reason, detailFlags);
    if (!ackQueue.push(ack))
    {
        Serial.println("FAIL: ack_queue_full");
        return;
    }

    Serial.print("ack queued command=");
    Serial.print(commandName(command.commandId));
    Serial.print(" command_seq=");
    Serial.print(command.commandSeq);
    Serial.print(" stage=");
    Serial.print(ackStageName(stage));
    Serial.print(" result=");
    Serial.println(resultName(result));
}

bool sendRawFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen, const char *label)
{
    uint8_t frame[nura::kMaxFrameLen];
    const uint16_t seq = downlinkSeq++;
    const size_t frameLen = nura::encodeFrame(type, seq, payload, payloadLen, frame, sizeof(frame));
    if (frameLen == 0U)
    {
        Serial.print("FAIL: encode_frame type=");
        Serial.println(label);
        return false;
    }

    LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
    LoRa.idle();
    delay(2);
    if (!LoRa.beginPacket())
    {
        LoRa.receive();
        Serial.print("FAIL: beginPacket type=");
        Serial.println(label);
        return false;
    }
    const size_t written = LoRa.write(frame, frameLen);
    const bool ok = written == frameLen && LoRa.endPacket() == 1;
    LoRa.receive();

    if (ok)
    {
        Serial.print("tx type=");
        Serial.print(label);
        Serial.print(" seq=");
        Serial.print(seq);
        Serial.print(" len=");
        Serial.println(frameLen);
    }
    else
    {
        Serial.print("FAIL: tx type=");
        Serial.println(label);
    }
    return ok;
}

bool sendAckIfQueued()
{
    nura::ControlPayload ack;
    if (!ackQueue.pop(ack))
    {
        return false;
    }

    uint8_t payload[nura::kControlPayloadLen];
    nura::encodeControlPayload(ack, payload, sizeof(payload));
    return sendRawFrame(nura::MESSAGE_CONTROL, payload, nura::kControlPayloadLen, "CONTROL");
}

uint16_t buildStatusWord()
{
    uint16_t status = nura::STATUS_LOW_IMU_OK |
                      nura::STATUS_HIGH_ACCEL_OK |
                      nura::STATUS_BARO_OK |
                      nura::STATUS_GPS_OK |
                      nura::STATUS_MAG_OK |
                      nura::STATUS_STORAGE_OK |
                      nura::STATUS_PYRO_CONTINUITY_OK |
                      nura::STATUS_RADIO_OK |
                      nura::STATUS_ARMED;
    if (deployFired)
    {
        status |= nura::STATUS_DEPLOY_FIRED;
    }
    return nura::statusWithFlightState(status, currentFlightState());
}

void sendFastTelemetry()
{
    const uint32_t nowMs = millis();
    const int16_t phase = static_cast<int16_t>((nowMs / 100UL) % 120UL);
    nura::FastTelemetry fast;
    fast.statusWord = buildStatusWord();
    fast.bootMs = nowMs;
    fast.baroDp2Pa = static_cast<int16_t>(-4 * phase);
    fast.lowAccelXCg = static_cast<int16_t>((phase % 20) - 10);
    fast.lowAccelYCg = static_cast<int16_t>(5 - (phase % 10));
    fast.lowAccelZCg = deployFired ? 65 : static_cast<int16_t>(100 + (phase % 18));
    fast.gyroXDps10 = static_cast<int16_t>((phase % 30) - 15);
    fast.gyroYDps10 = static_cast<int16_t>((phase % 26) - 13);
    fast.gyroZDps10 = static_cast<int16_t>((phase % 22) - 11);
    fast.battMv = static_cast<uint16_t>(12000U - static_cast<uint16_t>((nowMs / 10000UL) % 500UL));

    uint8_t payload[nura::kFastPayloadLen];
    nura::encodeFastPayload(fast, payload, sizeof(payload));
    sendRawFrame(nura::MESSAGE_FAST_TLM, payload, nura::kFastPayloadLen, "FAST");
}

void sendGpsTelemetry()
{
    const uint32_t nowMs = millis();
    const int32_t drift = static_cast<int32_t>((nowMs / 1000UL) % 100UL);
    nura::GpsTelemetry gps;
    gps.latitudeE7 = 371234567L + drift;
    gps.longitudeE7 = 1271234567L + drift;
    gps.gpsAltDm = static_cast<int16_t>(500 + (drift % 30));
    gps.speedCms = deployFired ? 420U : 1200U;
    gps.courseCdeg = 8520U;
    gps.hdopX10 = 12U;
    gps.satellites = 9U;
    gps.fixFlags = 0x1FU;
    gps.age100Ms = 1U;

    uint8_t payload[nura::kGpsPayloadLen];
    nura::encodeGpsPayload(gps, payload, sizeof(payload));
    sendRawFrame(nura::MESSAGE_GPS_TLM, payload, nura::kGpsPayloadLen, "GPS");
}

void handleCommand(const nura::ParsedFrame &frame, const nura::ControlPayload &command)
{
    if (command.subtype != nura::CONTROL_CMD)
    {
        return;
    }

    Serial.print("rx control subtype=CMD command=");
    Serial.print(commandName(command.commandId));
    Serial.print(" command_seq=");
    Serial.print(command.commandSeq);
    Serial.print(" frame_seq=");
    Serial.println(frame.seq);

    if (!nura::verifyControlAuthTag(command, frame.seq, kAuthKey))
    {
        enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_AUTH_FAILED, nura::REJECT_AUTH_TAG_MISMATCH);
        return;
    }

    const uint32_t nowMs = millis();
    if (command.validUntilMs != 0UL && static_cast<int32_t>(nowMs - command.validUntilMs) > 0)
    {
        enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_EXPIRED, nura::REJECT_COMMAND_EXPIRED);
        return;
    }

    if (wasRecentlyProcessed(command))
    {
        enqueueAck(command, nura::ACK_DUPLICATE, nura::RESULT_ALREADY_DONE, nura::REJECT_NONE);
        return;
    }

    switch (command.commandId)
    {
    case nura::COMMAND_FORCE_DEPLOY_RECOVERY:
        if (command.param0 != 0)
        {
            enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_BAD_FORMAT, nura::REJECT_DEPLOYMENT_INHIBITED);
            return;
        }
        enqueueAck(command, nura::ACK_ACCEPTED, nura::RESULT_OK, nura::REJECT_NONE);
        deployFired = true;
        rememberCommand(command);
        enqueueAck(command, nura::ACK_EXECUTED, nura::RESULT_OK, nura::REJECT_NONE);
        break;

    case nura::COMMAND_ABORT_PROPULSION_DEPRECATED:
        rememberCommand(command);
        enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_NOT_SUPPORTED, nura::REJECT_DEPRECATED_COMMAND);
        break;

    default:
        enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_NOT_SUPPORTED, nura::REJECT_UNKNOWN_COMMAND);
        break;
    }
}

void receiveCommands()
{
    const int packetSize = LoRa.parsePacket();
    if (packetSize <= 0)
    {
        return;
    }

    while (LoRa.available())
    {
        const int value = LoRa.read();
        if (value < 0)
        {
            break;
        }

        nura::ParsedFrame frame;
        if (!parser.feed(static_cast<uint8_t>(value), frame))
        {
            continue;
        }

        if (frame.type == nura::MESSAGE_CONTROL)
        {
            nura::ControlPayload control;
            if (nura::decodeControlPayload(frame.payload, frame.payloadLen, control))
            {
                handleCommand(frame, control);
            }
        }
    }
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    while (!Serial && millis() < 4000UL)
    {
    }

    Serial.println();
    Serial.println("NURA V1 Lite sender avionics emulator");
    Serial.println("role=sender board=teensy41 protocol=v1_lite");
    Serial.println("packet_set=FAST,GPS,CONTROL");
    Serial.println("rf=freq433_sf7_bw125_cr45_dev");

    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: sender radio init failed");
        return;
    }

    Serial.println("PASS: sender radio init OK");
    lastFastMs = millis();
    lastGpsMs = millis();
    sendFastTelemetry();
    sendGpsTelemetry();
}

void loop()
{
    if (!radioReady)
    {
        return;
    }

    receiveCommands();
    if (sendAckIfQueued())
    {
        return;
    }

    const uint32_t nowMs = millis();
    if ((nowMs - lastFastMs) >= kFastIntervalMs)
    {
        lastFastMs = nowMs;
        sendFastTelemetry();
        return;
    }

    if ((nowMs - lastGpsMs) >= kGpsIntervalMs)
    {
        lastGpsMs = nowMs;
        sendGpsTelemetry();
        return;
    }
}

