#include "telemetry_task.h"

#include <math.h>

#include "board_pinmap.h"

namespace
{
    constexpr float kGravity = 9.80665f;
    constexpr uint8_t kAuthKey[16] = {
        0x4e, 0x55, 0x52, 0x41, 0x2d, 0x56, 0x31, 0x4c,
        0x49, 0x54, 0x45, 0x2d, 0x54, 0x45, 0x53, 0x54};

    int16_t clampFloatToI16(float value)
    {
        if (!isfinite(value))
        {
            return 0;
        }
        if (value > 32767.0f)
        {
            return 32767;
        }
        if (value < -32768.0f)
        {
            return -32768;
        }
        return static_cast<int16_t>(value >= 0.0f ? value + 0.5f : value - 0.5f);
    }

    uint16_t clampFloatToU16(float value)
    {
        if (!isfinite(value) || value <= 0.0f)
        {
            return 0U;
        }
        if (value > 65535.0f)
        {
            return 65535U;
        }
        return static_cast<uint16_t>(value + 0.5f);
    }

    int32_t clampDoubleToI32(double value)
    {
        if (!isfinite(value))
        {
            return 0L;
        }
        if (value > 2147483647.0)
        {
            return 2147483647L;
        }
        if (value < -2147483648.0)
        {
            return -2147483647L - 1L;
        }
        return static_cast<int32_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }

    uint8_t clampU8FromU32(uint32_t value)
    {
        return value > 255UL ? 255U : static_cast<uint8_t>(value);
    }

    bool sampleFresh(uint32_t sampleMs, uint32_t nowMs, uint32_t freshMs)
    {
        return sampleMs != 0UL && (nowMs - sampleMs) <= freshMs;
    }
}

bool TelemetryTask::AckQueue::push(const nura::ControlPayload &item)
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

bool TelemetryTask::AckQueue::pop(nura::ControlPayload &out)
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

TelemetryTask::TelemetryTask(Sx127xLoRaHAL &radio,
                             const ImuState &imuState,
                             const GpsState &gpsState,
                             TelemetryState &telemetryState,
                             FlightState &flightState,
                             const AbortState &abortState,
                             Logger &logger,
                             const IAppConfig &config)
    : radio_(radio),
      imuState_(imuState),
      gpsState_(gpsState),
      telemetryState_(telemetryState),
      flightState_(flightState),
      abortState_(abortState),
      logger_(logger),
      config_(config)
{
}

const char *TelemetryTask::name() const
{
    return "telemetry";
}

bool TelemetryTask::init()
{
    radioReady_ = radio_.begin(buildRadioConfig());
    if (!radioReady_)
    {
        LOGE(logger_, 0U, "telemetry", "lora init failed");
        return false;
    }

    telemetryState_.health.deployFired = false;
    LOGI(logger_, 0U, "telemetry", "lora initialized");
    return true;
}

bool TelemetryTask::tick(uint32_t nowMs)
{
    if (!radioReady_)
    {
        return true;
    }

    receiveControl(nowMs);
    if (sendAckIfQueued())
    {
        return true;
    }

    if (!sentFast_ || (nowMs - lastFastTxMs_) >= config_.telemetryFastPeriodMs())
    {
        return sendFastTelemetry(nowMs);
    }

    if (!sentGps_ || (nowMs - lastGpsTxMs_) >= config_.telemetryGpsPeriodMs())
    {
        return sendGpsTelemetry(nowMs);
    }

    return true;
}

uint32_t TelemetryTask::periodMs() const
{
    return config_.telemetryTaskPeriodMs();
}

bool TelemetryTask::receiveControl(uint32_t nowMs)
{
    uint8_t buffer[nura::kMaxFrameLen];
    Sx127xLoRaPacket packet;
    if (!radio_.receive(buffer, sizeof(buffer), packet))
    {
        return false;
    }

    bool handled = false;
    for (size_t i = 0U; i < packet.length; ++i)
    {
        nura::ParsedFrame frame;
        if (!parser_.feed(buffer[i], frame) || frame.type != nura::MESSAGE_CONTROL)
        {
            continue;
        }

        nura::ControlPayload control;
        if (nura::decodeControlPayload(frame.payload, frame.payloadLen, control))
        {
            handleCommand(frame, control, nowMs);
            handled = true;
        }
    }
    return handled;
}

bool TelemetryTask::sendAckIfQueued()
{
    nura::ControlPayload ack;
    if (!ackQueue_.pop(ack))
    {
        return false;
    }

    uint8_t payload[nura::kControlPayloadLen];
    if (!nura::encodeControlPayload(ack, payload, sizeof(payload)))
    {
        return false;
    }
    return sendRawFrame(nura::MESSAGE_CONTROL, payload, nura::kControlPayloadLen);
}

bool TelemetryTask::sendFastTelemetry(uint32_t nowMs)
{
    const nura::FastTelemetry fast = buildFastTelemetry(nowMs);
    uint8_t payload[nura::kFastPayloadLen];
    if (!nura::encodeFastPayload(fast, payload, sizeof(payload)))
    {
        return false;
    }

    const bool ok = sendRawFrame(nura::MESSAGE_FAST_TLM, payload, nura::kFastPayloadLen);
    if (ok)
    {
        lastFastTxMs_ = nowMs;
        sentFast_ = true;
    }
    return true;
}

bool TelemetryTask::sendGpsTelemetry(uint32_t nowMs)
{
    const nura::GpsTelemetry gps = buildGpsTelemetry(nowMs);
    uint8_t payload[nura::kGpsPayloadLen];
    if (!nura::encodeGpsPayload(gps, payload, sizeof(payload)))
    {
        return false;
    }

    const bool ok = sendRawFrame(nura::MESSAGE_GPS_TLM, payload, nura::kGpsPayloadLen);
    if (ok)
    {
        lastGpsTxMs_ = nowMs;
        sentGps_ = true;
    }
    return true;
}

bool TelemetryTask::sendRawFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen)
{
    uint8_t frame[nura::kMaxFrameLen];
    const size_t frameLen = nura::encodeFrame(type, downlinkSeq_++, payload, payloadLen, frame, sizeof(frame));
    if (frameLen == 0U)
    {
        LOGW(logger_, 0U, "telemetry", "frame encode failed");
        return false;
    }

    if (!radio_.send(frame, frameLen))
    {
        LOGW(logger_, 0U, "telemetry", "lora tx failed");
        return false;
    }
    return true;
}

void TelemetryTask::handleCommand(const nura::ParsedFrame &frame, const nura::ControlPayload &command, uint32_t nowMs)
{
    if (command.subtype != nura::CONTROL_CMD)
    {
        return;
    }

    if (!nura::verifyControlAuthTag(command, frame.seq, kAuthKey))
    {
        enqueueAck(command, nura::ACK_REJECTED, nura::RESULT_AUTH_FAILED, nura::REJECT_AUTH_TAG_MISMATCH);
        return;
    }

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
        telemetryState_.health.deployFired = true;
        flightState_.state = State::DESCENT;
        flightState_.stateEnteredMs = nowMs;
        rememberCommand(command);
        enqueueAck(command, nura::ACK_EXECUTED, nura::RESULT_OK, nura::REJECT_NONE);
        LOGW(logger_, nowMs, "telemetry", "force deploy command executed");
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

void TelemetryTask::enqueueAck(const nura::ControlPayload &command,
                               uint8_t stage,
                               uint8_t result,
                               uint8_t reason,
                               uint16_t detailFlags)
{
    nura::ControlPayload ack;
    ack.subtype = nura::CONTROL_ACK;
    ack.commandId = command.commandId;
    ack.commandSeq = command.commandSeq;
    ack.nonce = command.nonce;
    ack.validUntilMs = millis();
    ack.authOrAck[0] = stage;
    ack.authOrAck[1] = result;
    ack.authOrAck[2] = reason;
    ack.authOrAck[3] = currentFlightStateCode();
    nura::writeU16(ack.authOrAck + 4, detailFlags);
    ack.authOrAck[6] = 0U;
    ack.authOrAck[7] = 0U;

    if (!ackQueue_.push(ack))
    {
        LOGW(logger_, 0U, "telemetry", "ack queue full");
    }
}

bool TelemetryTask::wasRecentlyProcessed(const nura::ControlPayload &command) const
{
    for (const RecentCommand &recent : recentCommands_)
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

void TelemetryTask::rememberCommand(const nura::ControlPayload &command)
{
    RecentCommand &slot = recentCommands_[recentCommandWriteIndex_];
    slot.valid = true;
    slot.commandId = command.commandId;
    slot.commandSeq = command.commandSeq;
    slot.nonce = command.nonce;
    recentCommandWriteIndex_ = static_cast<uint8_t>((recentCommandWriteIndex_ + 1U) % 4U);
}

nura::FastTelemetry TelemetryTask::buildFastTelemetry(uint32_t nowMs) const
{
    nura::FastTelemetry fast;
    fast.statusWord = buildStatusWord(nowMs);
    fast.bootMs = nowMs;

    const BarometerTelemetryData &baro = telemetryState_.barometer;
    if (baro.valid && baro.referenceValid)
    {
        fast.baroDp2Pa = clampFloatToI16((baro.pressurePa - baro.referencePressurePa) * 0.5f);
    }

    const ImuData &imu = imuState_.data;
    fast.lowAccelXCg = clampFloatToI16((imu.accelXMps2 / kGravity) * 100.0f);
    fast.lowAccelYCg = clampFloatToI16((imu.accelYMps2 / kGravity) * 100.0f);
    fast.lowAccelZCg = clampFloatToI16((imu.accelZMps2 / kGravity) * 100.0f);
    fast.gyroXDps10 = clampFloatToI16(imu.gyroXDps * 10.0f);
    fast.gyroYDps10 = clampFloatToI16(imu.gyroYDps * 10.0f);
    fast.gyroZDps10 = clampFloatToI16(imu.gyroZDps * 10.0f);
    fast.battMv = telemetryState_.power.batteryMv;
    return fast;
}

nura::GpsTelemetry TelemetryTask::buildGpsTelemetry(uint32_t nowMs) const
{
    (void)nowMs;
    const GpsData &gnss = gpsState_.data;
    nura::GpsTelemetry gps;
    gps.latitudeE7 = clampDoubleToI32(gnss.latitudeDeg * 10000000.0);
    gps.longitudeE7 = clampDoubleToI32(gnss.longitudeDeg * 10000000.0);
    gps.gpsAltDm = clampFloatToI16(static_cast<float>(gnss.altitudeM * 10.0));
    gps.speedCms = clampFloatToU16(static_cast<float>(gnss.speedMps * 100.0));
    gps.courseCdeg = clampFloatToU16(static_cast<float>(gnss.courseDeg * 100.0));
    gps.hdopX10 = clampU8FromU32(static_cast<uint32_t>(gnss.hdop * 10.0));
    gps.satellites = clampU8FromU32(gnss.satellites);
    gps.fixFlags = static_cast<uint8_t>((gnss.charsProcessed > 0UL ? 0x01U : 0U) | (gnss.hasFix ? 0x02U : 0U));
    gps.age100Ms = clampU8FromU32(gnss.locationAgeMs / 100UL);
    return gps;
}

uint16_t TelemetryTask::buildStatusWord(uint32_t nowMs) const
{
    const uint32_t freshMs = config_.telemetrySensorFreshMs();
    uint16_t status = 0U;

    if (sampleFresh(imuState_.data.lastUpdatedMs, nowMs, freshMs))
    {
        status |= nura::STATUS_LOW_IMU_OK;
    }
    if (telemetryState_.health.highAccelOk)
    {
        status |= nura::STATUS_HIGH_ACCEL_OK;
    }
    if (telemetryState_.barometer.valid && sampleFresh(telemetryState_.barometer.lastUpdatedMs, nowMs, freshMs))
    {
        status |= nura::STATUS_BARO_OK;
    }
    if (gpsState_.data.hasFix &&
        sampleFresh(gpsState_.data.lastUpdatedMs, nowMs, config_.gnssMaxFixAgeMs()))
    {
        status |= nura::STATUS_GPS_OK;
    }
    if (telemetryState_.health.magOk)
    {
        status |= nura::STATUS_MAG_OK;
    }
    if (telemetryState_.health.storageOk)
    {
        status |= nura::STATUS_STORAGE_OK;
    }
    if (telemetryState_.health.pyroContinuityOk)
    {
        status |= nura::STATUS_PYRO_CONTINUITY_OK;
    }
    if (radioReady_)
    {
        status |= nura::STATUS_RADIO_OK;
    }
    if (flightState_.state == State::ARMED || flightState_.state == State::LAUNCH || flightState_.state == State::DESCENT)
    {
        status |= nura::STATUS_ARMED;
    }
    if (abortState_.status.active)
    {
        status |= nura::STATUS_ABORT_ACTIVE;
    }
    if (telemetryState_.health.deployFired)
    {
        status |= nura::STATUS_DEPLOY_FIRED;
    }
    if (abortState_.status.active || flightState_.state == State::SAFE)
    {
        status |= nura::STATUS_CRITICAL_FAULT;
    }

    return nura::statusWithFlightState(status, currentFlightStateCode());
}

uint8_t TelemetryTask::currentFlightStateCode() const
{
    switch (flightState_.state)
    {
    case State::BOOT:
        return nura::FLIGHT_BOOT;
    case State::IDLE:
        return nura::FLIGHT_IDLE;
    case State::ARMED:
        return nura::FLIGHT_ARMED;
    case State::LAUNCH:
        return nura::FLIGHT_LAUNCH;
    case State::DESCENT:
        return nura::FLIGHT_DESCENT;
    case State::GROUND:
        return nura::FLIGHT_GROUND;
    case State::SAFE:
        return nura::FLIGHT_SAFE;
    default:
        return nura::FLIGHT_SAFE;
    }
}

Sx127xLoRaConfig TelemetryTask::buildRadioConfig() const
{
    Sx127xLoRaConfig radioConfig;
    radioConfig.frequencyHz = config_.loraFrequencyHz();
    radioConfig.ssPin = BoardPinMap::Ra01DevelopmentLoRa::ssPin;
    radioConfig.resetPin = BoardPinMap::Ra01DevelopmentLoRa::resetPin;
    radioConfig.libraryResetPin = BoardPinMap::Ra01DevelopmentLoRa::libraryResetPin;
    radioConfig.dio0Pin = BoardPinMap::Ra01DevelopmentLoRa::dio0Pin;
    radioConfig.spiFrequency = config_.loraSpiFrequencyHz();
    radioConfig.txPowerDbm = config_.loraTxPowerDbm();
    radioConfig.spreadingFactor = config_.loraSpreadingFactor();
    radioConfig.signalBandwidthHz = config_.loraSignalBandwidthHz();
    radioConfig.codingRateDenominator = config_.loraCodingRateDenominator();
    radioConfig.preambleLength = config_.loraPreambleLength();
    radioConfig.syncWord = config_.loraSyncWord();
    radioConfig.initAttempts = config_.loraInitAttempts();
    radioConfig.spiMode = config_.loraSpiMode();
    radioConfig.probeSpiMode = config_.loraProbeSpiMode();
    radioConfig.crcEnabled = true;
    return radioConfig;
}
