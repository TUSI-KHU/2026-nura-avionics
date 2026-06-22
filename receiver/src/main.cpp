#include <Arduino.h>
#include <SPI.h>

#include "board_pinmap.h"
#include "nura_constants.h"
#include "nura_protocol_v1_lite.h"

#define private public
#include <LoRa.h>
#undef private

namespace
{
constexpr unsigned long kSerialBaud = 115200UL;
#if defined(NURA_GROUND_SX1276)
constexpr long kLoraFrequencyHz = NuraConstants::LoRa::kFlightFrequencyHz;
#else
constexpr long kLoraFrequencyHz = 433000000L;
#endif
constexpr uint32_t kLoraSpiFrequencyHz = 125000UL;
constexpr uint8_t kLoraSsPin = 10U;
constexpr uint8_t kLoraResetPin = 9U;
constexpr int8_t kLoraLibraryResetPin = -1;
constexpr uint8_t kLoraDio0Pin = 2U;
constexpr uint8_t kLoraRxEnablePin = 4U;
constexpr uint8_t kLoraTxEnablePin = 3U;
constexpr int kLoraTxPowerDbm = 10;
constexpr int kLoraSpreadingFactor = 7;
constexpr long kLoraSignalBandwidthHz = 125000L;
constexpr int kLoraCodingRateDenominator = 5;
constexpr long kLoraPreambleLength = 8L;
constexpr int kLoraSyncWord = 0x12;
constexpr uint8_t kLoraRegVersion = 0x42U;
constexpr uint8_t kLoraRegRssiValue = 0x1BU;
constexpr int16_t kLoraHfRssiOffsetDbm = -157;
constexpr uint8_t kLoraExpectedVersion = 0x12U;
constexpr uint8_t kLoraInitAttempts = 5U;
constexpr uint32_t kRssiSampleIntervalMs = 5UL;

constexpr uint32_t kCommandRetryIntervalMs = 250UL;
constexpr uint8_t kCommandMaxAttempts = 8U;

#if defined(NURA_RECEIVER_AUTO_COMMAND_TEST)
constexpr bool kAutoCommandTestEnabled = true;
#else
constexpr bool kAutoCommandTestEnabled = false;
#endif

uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0U;
uint8_t lastMode0Version = 0U;
uint8_t lastMode1Version = 0U;
uint8_t lastMode2Version = 0U;
uint8_t lastMode3Version = 0U;
bool radioReady = false;
uint16_t uplinkFrameSeq = 0U;
uint16_t nextCommandSeq = 1U;
uint32_t fastRxCount = 0UL;
uint32_t gpsRxCount = 0UL;
uint32_t controlAckRxCount = 0UL;
uint32_t phyRxCount = 0UL;
uint32_t frameDecodeFailCount = 0UL;
bool forceDeployDone = false;
bool deprecatedAbortDone = false;
bool completionPrinted = false;
uint32_t forceDeployDoneMs = 0UL;
uint32_t lastStatusMs = 0UL;
uint32_t lastRssiSampleMs = 0UL;
int16_t currentRssiDbm = -200;
int16_t peakRssiDbm = -200;

struct PendingCommand
{
    bool active = false;
    uint8_t commandId = 0U;
    uint16_t commandSeq = 0U;
    uint16_t frameSeq = 0U;
    uint32_t nonce = 0UL;
    int16_t param0 = 0;
    int16_t param1 = 0;
    uint8_t attempts = 0U;
    uint32_t lastTxMs = 0UL;
};

PendingCommand pending;

void beginSpi()
{
    SPI.begin();
}

void configureRfSwitchForReceive()
{
#if defined(NURA_GROUND_SX1276)
    pinMode(kLoraRxEnablePin, OUTPUT);
    pinMode(kLoraTxEnablePin, OUTPUT);
    digitalWrite(kLoraTxEnablePin, LOW);
    digitalWrite(kLoraRxEnablePin, HIGH);
#endif
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
    pinMode(kLoraSsPin, OUTPUT);
    digitalWrite(kLoraSsPin, HIGH);
    SPI.beginTransaction(settings);
    digitalWrite(kLoraSsPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(address & 0x7FU);
    const uint8_t value = SPI.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(kLoraSsPin, HIGH);
    SPI.endTransaction();
    return value;
}

void resetRadio()
{
    pinMode(kLoraSsPin, OUTPUT);
    digitalWrite(kLoraSsPin, HIGH);
    pinMode(kLoraResetPin, OUTPUT);
    digitalWrite(kLoraResetPin, LOW);
    delay(50);
    digitalWrite(kLoraResetPin, HIGH);
    delay(500);
}

bool beginRadio()
{
    LoRa.setPins(kLoraSsPin, kLoraLibraryResetPin, kLoraDio0Pin);
    LoRa.setSPIFrequency(kLoraSpiFrequencyHz);

    for (uint8_t attempt = 1U; attempt <= kLoraInitAttempts; ++attempt)
    {
        beginSpi();
        resetRadio();

        lastMode0Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE0);
#if !defined(NURA_GROUND_SX1276)
        lastMode1Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE1);
        lastMode2Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE2);
        lastMode3Version = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE3);
#endif

        Serial.print("init_attempt=");
        Serial.print(attempt);
        Serial.print(" m0=");
        printHexByte(lastMode0Version);
        Serial.print(" m1=");
        printHexByte(lastMode1Version);
        Serial.print(" m2=");
        printHexByte(lastMode2Version);
        Serial.print(" m3=");
        printHexByte(lastMode3Version);
        Serial.println();

#if defined(NURA_GROUND_SX1276)
        if (lastMode0Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else
        {
            delay(250);
            continue;
        }
#else
        if (lastMode1Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE1;
            selectedSpiModeNumber = 1U;
        }
        else if (lastMode0Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else if (lastMode2Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE2;
            selectedSpiModeNumber = 2U;
        }
        else if (lastMode3Version == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE3;
            selectedSpiModeNumber = 3U;
        }
        else
        {
            delay(250);
            continue;
        }
#endif

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
            LoRa.setPreambleLength(kLoraPreambleLength);
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

const char *flightStateName(uint8_t state)
{
    switch (state)
    {
    case nura::FLIGHT_INIT:
        return "INIT";
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
    case nura::FLIGHT_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
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

const char *stageName(uint8_t stage)
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
    case nura::RESULT_BAD_FORMAT:
        return "BAD_FORMAT";
    case nura::RESULT_BAD_STATE:
        return "BAD_STATE";
    case nura::RESULT_NOT_ARMED:
        return "NOT_ARMED";
    case nura::RESULT_ALREADY_DONE:
        return "ALREADY_DONE";
    case nura::RESULT_NOT_SUPPORTED:
        return "NOT_SUPPORTED";
    case nura::RESULT_ACTUATOR_FAULT:
        return "ACTUATOR_FAULT";
    case nura::RESULT_INTERNAL_ERROR:
        return "INTERNAL_ERROR";
    default:
        return "UNKNOWN";
    }
}

const char *reasonName(uint8_t reason)
{
    switch (reason)
    {
    case nura::REJECT_NONE:
        return "NONE";
    case nura::REJECT_COMMAND_EXPIRED:
        return "COMMAND_EXPIRED";
    case nura::REJECT_UNKNOWN_COMMAND:
        return "UNKNOWN_COMMAND";
    case nura::REJECT_AUTH_TAG_MISMATCH:
        return "AUTH_TAG_MISMATCH";
    case nura::REJECT_DUPLICATE_OLDER_COMMAND:
        return "DUPLICATE_OLDER_COMMAND";
    case nura::REJECT_DEPLOYMENT_INHIBITED:
        return "DEPLOYMENT_INHIBITED";
    case nura::REJECT_CONTINUITY_BAD:
        return "CONTINUITY_BAD";
    case nura::REJECT_STATE_REJECTED:
        return "STATE_REJECTED";
    case nura::REJECT_DEPRECATED_COMMAND:
        return "DEPRECATED_COMMAND";
    case nura::REJECT_PROFILE_REJECTED:
        return "PROFILE_REJECTED";
    default:
        return "UNKNOWN";
    }
}

bool sendControlCommand(PendingCommand &cmd)
{
    nura::ControlPayload control;
    control.subtype = nura::CONTROL_CMD;
    control.commandId = cmd.commandId;
    control.commandSeq = cmd.commandSeq;
    control.nonce = cmd.nonce;
    control.validUntilMs = 0UL;
    control.param0 = cmd.param0;
    control.param1 = cmd.param1;
    nura::makeControlAuthTag(control,
                             cmd.frameSeq,
                             NuraConstants::Telemetry::kControlAuthKey,
                             control.authOrAck);

    uint8_t payload[nura::kControlPayloadLen];
    uint8_t frame[nura::kMaxFrameLen];
    nura::encodeControlPayload(control, payload, sizeof(payload));
    const size_t frameLen = nura::encodeFrame(nura::MESSAGE_CONTROL,
                                              NuraConstants::Telemetry::kVehicleId,
                                              cmd.frameSeq,
                                              nura::FrameDirection::UPLINK,
                                              NuraConstants::Telemetry::kControlAuthKey,
                                              payload,
                                              nura::kControlPayloadLen,
                                              frame,
                                              sizeof(frame));
    if (frameLen == 0U)
    {
        Serial.println("FAIL: control_encode");
        return false;
    }

    LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
    LoRa.idle();
    delay(2);
    if (!LoRa.beginPacket())
    {
        LoRa.receive();
        Serial.println("FAIL: control_beginPacket");
        return false;
    }
    const size_t written = LoRa.write(frame, frameLen);
    const bool ok = written == frameLen && LoRa.endPacket() == 1;
    LoRa.receive();

    ++cmd.attempts;
    cmd.lastTxMs = millis();

    Serial.print("cmd tx command=");
    Serial.print(commandName(cmd.commandId));
    Serial.print(" command_seq=");
    Serial.print(cmd.commandSeq);
    Serial.print(" frame_seq=");
    Serial.print(cmd.frameSeq);
    Serial.print(" attempt=");
    Serial.print(cmd.attempts);
    Serial.print(" len=");
    Serial.println(frameLen);
    if (!ok)
    {
        Serial.println("FAIL: control_tx");
    }
    return ok;
}

void startCommand(uint8_t commandId, int16_t param0, int16_t param1)
{
    pending = PendingCommand{};
    pending.active = true;
    pending.commandId = commandId;
    pending.commandSeq = nextCommandSeq++;
    pending.frameSeq = uplinkFrameSeq++;
    pending.nonce = 0x4E550000UL ^ (static_cast<uint32_t>(pending.commandSeq) << 8) ^ millis();
    pending.param0 = param0;
    pending.param1 = param1;
    pending.lastTxMs = 0UL;
    pending.attempts = 0U;

    Serial.print("cmd start command=");
    Serial.print(commandName(commandId));
    Serial.print(" command_seq=");
    Serial.println(pending.commandSeq);
    sendControlCommand(pending);
}

void serviceCommandSender()
{
    if (!kAutoCommandTestEnabled)
    {
        return;
    }

    if (!pending.active)
    {
        if (!forceDeployDone && fastRxCount >= 10UL && gpsRxCount >= 1UL && millis() > 3000UL)
        {
            startCommand(nura::COMMAND_FORCE_DEPLOY_RECOVERY, 0, 0);
        }
        else if (forceDeployDone && !deprecatedAbortDone && (millis() - forceDeployDoneMs) > 1500UL)
        {
            startCommand(nura::COMMAND_ABORT_PROPULSION_DEPRECATED, 0, 0);
        }
        return;
    }

    if (pending.attempts >= kCommandMaxAttempts)
    {
        Serial.print("FAIL: command_timeout command=");
        Serial.print(commandName(pending.commandId));
        Serial.print(" command_seq=");
        Serial.println(pending.commandSeq);
        pending.active = false;
        return;
    }

    if ((millis() - pending.lastTxMs) >= kCommandRetryIntervalMs)
    {
        sendControlCommand(pending);
    }
}

void handleFast(const nura::ParsedFrame &frame)
{
    nura::FastTelemetry fast;
    if (!nura::decodeFastPayload(frame.payload, frame.payloadLen, fast))
    {
        Serial.println("FAIL: fast_decode");
        return;
    }
    ++fastRxCount;

    const uint8_t stateCode = nura::flightStateFromStatus(fast.statusWord);
    const float accelXG = static_cast<float>(fast.lowAccelXCg) * 0.01f;
    const float accelYG = static_cast<float>(fast.lowAccelYCg) * 0.01f;
    const float accelZG = static_cast<float>(fast.lowAccelZCg) * 0.01f;
    const float gyroXDps = static_cast<float>(fast.gyroXDps10) * 0.1f;
    const float gyroYDps = static_cast<float>(fast.gyroYDps10) * 0.1f;
    const float gyroZDps = static_cast<float>(fast.gyroZDps10) * 0.1f;

    Serial.print("rx type=FAST seq=");
    Serial.print(frame.seq);
    Serial.print(" boot_ms=");
    Serial.print(static_cast<unsigned long>(fast.bootMs));
    Serial.print(" state=");
    Serial.print(flightStateName(stateCode));
    Serial.print(" state_code=");
    Serial.print(stateCode);
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
    Serial.print(accelXG, 2);
    Serial.print(',');
    Serial.print(accelYG, 2);
    Serial.print(',');
    Serial.print(accelZG, 2);
    Serial.print(')');
    Serial.print(" gyro_dps=(");
    Serial.print(gyroXDps, 1);
    Serial.print(',');
    Serial.print(gyroYDps, 1);
    Serial.print(',');
    Serial.print(gyroZDps, 1);
    Serial.print(')');
    Serial.print(" batt_mv=");
    Serial.print(fast.battMv);
    Serial.print(" health=");
    Serial.print((fast.statusWord & nura::STATUS_LOW_IMU_OK) != 0U ? "imu" : "-");
    Serial.print(',');
    Serial.print((fast.statusWord & nura::STATUS_BARO_OK) != 0U ? "baro" : "-");
    Serial.print(',');
    Serial.print((fast.statusWord & nura::STATUS_GPS_OK) != 0U ? "gps" : "-");
    Serial.print(',');
    Serial.print((fast.statusWord & nura::STATUS_RADIO_OK) != 0U ? "radio" : "-");
    Serial.print(" rssi=");
    Serial.print(LoRa.packetRssi());
    Serial.print(" snr=");
    Serial.println(LoRa.packetSnr());
}

void handleGps(const nura::ParsedFrame &frame)
{
    nura::GpsTelemetry gps;
    if (!nura::decodeGpsPayload(frame.payload, frame.payloadLen, gps))
    {
        Serial.println("FAIL: gps_decode");
        return;
    }
    ++gpsRxCount;

    const double latitudeDeg = static_cast<double>(gps.latitudeE7) * 0.0000001;
    const double longitudeDeg = static_cast<double>(gps.longitudeE7) * 0.0000001;
    const float altitudeM = static_cast<float>(gps.gpsAltDm) * 0.1f;
    const float speedMps = static_cast<float>(gps.speedCms) * 0.01f;
    const float courseDeg = static_cast<float>(gps.courseCdeg) * 0.01f;
    const float hdop = static_cast<float>(gps.hdopX10) * 0.1f;
    const float ageS = static_cast<float>(gps.age100Ms) * 0.1f;

    Serial.print("rx type=GPS seq=");
    Serial.print(frame.seq);
    Serial.print(" fix=");
    Serial.print((gps.fixFlags & 0x02U) != 0U ? "yes" : "no");
    Serial.print(" lat_e7=");
    Serial.print(static_cast<long>(gps.latitudeE7));
    Serial.print(" lon_e7=");
    Serial.print(static_cast<long>(gps.longitudeE7));
    Serial.print(" lat_deg=");
    Serial.print(latitudeDeg, 7);
    Serial.print(" lon_deg=");
    Serial.print(longitudeDeg, 7);
    Serial.print(" alt_m=");
    Serial.print(altitudeM, 1);
    Serial.print(" speed_mps=");
    Serial.print(speedMps, 2);
    Serial.print(" course_deg=");
    Serial.print(courseDeg, 2);
    Serial.print(" hdop=");
    Serial.print(hdop, 1);
    Serial.print(" sats=");
    Serial.print(gps.satellites);
    Serial.print(" age_s=");
    Serial.print(ageS, 1);
    Serial.print(" rssi=");
    Serial.print(LoRa.packetRssi());
    Serial.print(" snr=");
    Serial.println(LoRa.packetSnr());
}

void handleControl(const nura::ParsedFrame &frame)
{
    nura::ControlPayload control;
    if (!nura::decodeControlPayload(frame.payload, frame.payloadLen, control))
    {
        Serial.println("FAIL: control_decode");
        return;
    }
    if (control.subtype != nura::CONTROL_ACK)
    {
        return;
    }

    ++controlAckRxCount;
    const uint8_t stage = control.authOrAck[0];
    const uint8_t result = control.authOrAck[1];
    const uint8_t reason = control.authOrAck[2];

    Serial.print("rx type=CONTROL subtype=ACK command=");
    Serial.print(commandName(control.commandId));
    Serial.print(" command_seq=");
    Serial.print(control.commandSeq);
    Serial.print(" frame_seq=");
    Serial.print(frame.seq);
    Serial.print(" stage=");
    Serial.print(stageName(stage));
    Serial.print(" result=");
    Serial.print(resultName(result));
    Serial.print(" reason=");
    Serial.println(reasonName(reason));

    if (!pending.active ||
        control.commandSeq != pending.commandSeq ||
        control.commandId != pending.commandId ||
        control.nonce != pending.nonce)
    {
        return;
    }

    if (pending.commandId == nura::COMMAND_FORCE_DEPLOY_RECOVERY &&
        stage == nura::ACK_EXECUTED &&
        result == nura::RESULT_OK)
    {
        forceDeployDone = true;
        forceDeployDoneMs = millis();
        pending.active = false;
        Serial.println("PASS: force_deploy_ack_executed");
    }
    else if (pending.commandId == nura::COMMAND_ABORT_PROPULSION_DEPRECATED &&
             stage == nura::ACK_REJECTED &&
             result == nura::RESULT_NOT_SUPPORTED)
    {
        deprecatedAbortDone = true;
        pending.active = false;
        Serial.println("PASS: deprecated_abort_rejected");
    }
    else if (stage == nura::ACK_REJECTED)
    {
        pending.active = false;
    }
}

void receiveFrames()
{
    const int packetSize = LoRa.parsePacket();
    if (packetSize <= 0)
    {
        return;
    }
    ++phyRxCount;

    uint8_t buffer[nura::kMaxFrameLen];
    size_t count = 0U;
    bool overflow = false;
    while (LoRa.available())
    {
        const int value = LoRa.read();
        if (value < 0)
        {
            break;
        }

        if (count < sizeof(buffer))
        {
            buffer[count] = static_cast<uint8_t>(value);
        }
        else
        {
            overflow = true;
        }
        ++count;
    }

    if (overflow || count != static_cast<size_t>(packetSize))
    {
        return;
    }

    nura::ParsedFrame frame;
    if (!nura::decodeFrame(buffer,
                           count,
                           NuraConstants::Telemetry::kVehicleId,
                           nura::FrameDirection::DOWNLINK,
                           NuraConstants::Telemetry::kControlAuthKey,
                           frame))
    {
        ++frameDecodeFailCount;
        return;
    }

    switch (frame.type)
    {
    case nura::MESSAGE_FAST_TLM:
        handleFast(frame);
        break;
    case nura::MESSAGE_GPS_TLM:
        handleGps(frame);
        break;
    case nura::MESSAGE_CONTROL:
        handleControl(frame);
        break;
    default:
        break;
    }
}

void printCompletionIfReady()
{
    if (!kAutoCommandTestEnabled)
    {
        return;
    }

    if (completionPrinted || !forceDeployDone || !deprecatedAbortDone)
    {
        return;
    }
    completionPrinted = true;
    Serial.print("PASS: v1_lite_pair_test_complete fast=");
    Serial.print(static_cast<unsigned long>(fastRxCount));
    Serial.print(" gps=");
    Serial.print(static_cast<unsigned long>(gpsRxCount));
    Serial.print(" control_ack=");
    Serial.println(static_cast<unsigned long>(controlAckRxCount));
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    while (!Serial && millis() < 4000UL)
    {
    }

    Serial.println();
    Serial.println("NURA V2 Lite authenticated receiver GCS emulator");
    Serial.println("role=receiver board=teensy41 protocol=v2_lite_auth");
    Serial.println("packet_set=FAST,GPS,CONTROL");
#if defined(NURA_GROUND_SX1276)
    Serial.println("rf=freq920900000_sf7_bw125_cr45_sx1276_ground");
#else
    Serial.println("rf=freq433_sf7_bw125_cr45_dev");
#endif
    Serial.print("identity=");
    Serial.println(NuraConstants::Telemetry::kRadioIdentityProvisioned ? "provisioned" : "public_bench_unsafe_for_flight");
    Serial.print("mode=");
    Serial.println(kAutoCommandTestEnabled ? "pair_test_auto_commands" : "telemetry_receive_only");

    configureRfSwitchForReceive();
    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: receiver radio init failed");
        return;
    }

    Serial.println("PASS: receiver radio init OK");
}

void loop()
{
    const uint32_t nowMs = millis();
    if (radioReady && (nowMs - lastRssiSampleMs) >= kRssiSampleIntervalMs)
    {
        lastRssiSampleMs = nowMs;
        currentRssiDbm = kLoraHfRssiOffsetDbm +
                         static_cast<int16_t>(LoRa.readRegister(kLoraRegRssiValue));
        if (currentRssiDbm > peakRssiDbm)
        {
            peakRssiDbm = currentRssiDbm;
        }
    }

    if ((nowMs - lastStatusMs) >= 1000UL)
    {
        lastStatusMs = nowMs;
        Serial.print("status radio=");
        Serial.print(radioReady ? "ready" : "not_ready");
        Serial.print(" spi_mode=");
        Serial.print(selectedSpiModeNumber);
        Serial.print(" m0=");
        printHexByte(lastMode0Version);
        Serial.print(" m1=");
        printHexByte(lastMode1Version);
        Serial.print(" m2=");
        printHexByte(lastMode2Version);
        Serial.print(" m3=");
        printHexByte(lastMode3Version);
        Serial.print(" phy_rx=");
        Serial.print(phyRxCount);
        Serial.print(" decode_fail=");
        Serial.print(frameDecodeFailCount);
        Serial.print(" rssi_now_dbm=");
        Serial.print(currentRssiDbm);
        Serial.print(" rssi_peak_dbm=");
        Serial.print(peakRssiDbm);
        Serial.println();
        peakRssiDbm = currentRssiDbm;
    }

    if (!radioReady)
    {
        return;
    }

    receiveFrames();
    serviceCommandSender();
    printCompletionIfReady();
}
