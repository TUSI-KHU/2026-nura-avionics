#include "flight_log_task.h"

#include <math.h>
#include <string.h>

#include "nura_constants.h"

namespace
{
constexpr uint8_t kFastFlagLowImuUpdated = 1U << 0;
constexpr uint8_t kFastFlagAttitudeValid = 1U << 1;
constexpr uint8_t kFastFlagTiltValid = 1U << 2;
constexpr uint8_t kFastFlagBaroValid = 1U << 3;
constexpr uint8_t kFastFlagBaroReferenceValid = 1U << 4;
constexpr uint8_t kFastFlagBaroFault = 1U << 5;
constexpr uint8_t kFastFlagGpsFix = 1U << 6;

constexpr uint8_t kGpsFlagValid = 1U << 0;
constexpr uint8_t kGpsFlagFix = 1U << 1;

constexpr uint16_t kHealthHighAccelOk = 1U << 0;
constexpr uint16_t kHealthMagOk = 1U << 1;
constexpr uint16_t kHealthStorageOk = 1U << 2;
constexpr uint16_t kHealthPyroContinuityOk = 1U << 3;
constexpr uint16_t kHealthDeployFired = 1U << 4;

constexpr int16_t kI16Max = 32767;
constexpr int16_t kI16Min = -32768;
constexpr int32_t kI32Max = 2147483647L;
constexpr int32_t kI32Min = static_cast<int32_t>(-2147483647L - 1L);
} // namespace

FlightLogTask::FlightLogTask(FlightState &flightState,
                             const ImuState &imuState,
                             const HighGImuState &highGImuState,
                             const MagnetometerState &magnetometerState,
                             const GpsState &gpsState,
                             TelemetryState &telemetryState,
                             IFlightLogStorage &storage,
                             Logger &logger)
    : flightState_(flightState),
      imuState_(imuState),
      highGImuState_(highGImuState),
      magnetometerState_(magnetometerState),
      gpsState_(gpsState),
      telemetryState_(telemetryState),
      storage_(storage),
      logger_(logger)
{
}

const char *FlightLogTask::name() const
{
    return "flight_log";
}

bool FlightLogTask::init()
{
    ramBuffer_.clear();
    nextSequence_ = 0U;
    lastFastSampleMs_ = 0U;
    lastSlowSampleMs_ = 0U;
    lastDroppedRecords_ = 0U;
    lastDroppedDecisionTraces_ = 0U;
    lastState_ = flightState_.state;
    flightState_.clearDecisionTraceQueue();
    flightState_.clearTransitionTraceQueue();
    stopped_ = false;

    storageStarted_ = storage_.begin();
    telemetryState_.health.storageOk = storageStarted_ && storage_.healthy();
    if (!telemetryState_.health.storageOk)
    {
        LOGE(logger_, 0U, "flight_log", "storage offline");
        return false;
    }

    LOGI(logger_, 0U, "flight_log", "storage online");

    enqueueEvent(nura_log::EventId::BOOT, 0U, flightState_.state, flightState_.state, 0U, 0U);
    return true;
}

bool FlightLogTask::tick(uint32_t nowMs)
{
    if (stopped_)
    {
        return true;
    }

    telemetryState_.health.storageOk = storageStarted_ && storage_.healthy();

    FlightStateTransitionTrace transition;
    while (flightState_.popTransitionTrace(transition))
    {
        enqueueEvent(nura_log::EventId::STATE_TRANSITION,
                     transition.timestampMs,
                     transition.previous,
                     transition.current,
                     transition.timestampMs,
                     0U);
        lastState_ = transition.current;
    }

    FlightDecisionTrace decision;
    while (flightState_.popDecisionTrace(decision))
    {
        enqueueDecision(decision);
    }

    if (lastFastSampleMs_ == 0U ||
        (nowMs - lastFastSampleMs_) >= NuraConstants::Logger::kFlightLogFastPeriodMs)
    {
        enqueueFastSample(nowMs);
        lastFastSampleMs_ = nowMs;
    }

    if (lastSlowSampleMs_ == 0U ||
        (nowMs - lastSlowSampleMs_) >= NuraConstants::Logger::kFlightLogSlowPeriodMs)
    {
        enqueueSlowSample(nowMs);
        lastSlowSampleMs_ = nowMs;
    }

    if (ramBuffer_.droppedRecords() != lastDroppedRecords_)
    {
        lastDroppedRecords_ = ramBuffer_.droppedRecords();
        LOGW(logger_, nowMs, "flight_log", "ram fifo dropped records");
    }
    if (flightState_.droppedDecisionTraces != lastDroppedDecisionTraces_)
    {
        lastDroppedDecisionTraces_ = flightState_.droppedDecisionTraces;
        LOGW(logger_, nowMs, "flight_log", "decision trace dropped");
    }

    drainToStorage(nowMs, NuraConstants::Logger::kFlightLogDrainRecordsPerTick);

    if (flightState_.state == State::GROUND)
    {
        stopAtGround(nowMs);
    }

    return true;
}

uint32_t FlightLogTask::periodMs() const
{
    return NuraConstants::Logger::kFlightLogFastPeriodMs;
}

bool FlightLogTask::enqueueFastSample(uint32_t nowMs)
{
    const ImuData &imu = imuState_.data;
    const BarometerTelemetryData &baro = telemetryState_.barometer;

    nura_log::FastSamplePayload payload;
    memset(&payload, 0, sizeof(payload));
    payload.state = static_cast<uint8_t>(flightState_.state);
    payload.flags = fastFlags();
    payload.healthFlags = healthFlags();
    payload.decisionSeq = flightState_.decisionTrace.seq;
    payload.lowImuUpdatedMs = imu.lastUpdatedMs;
    payload.lowAccelMg[0] = scaledFloatToI16(imu.accelXMps2 / NuraConstants::Physics::kGravityMps2, 1000.0f);
    payload.lowAccelMg[1] = scaledFloatToI16(imu.accelYMps2 / NuraConstants::Physics::kGravityMps2, 1000.0f);
    payload.lowAccelMg[2] = scaledFloatToI16(imu.accelZMps2 / NuraConstants::Physics::kGravityMps2, 1000.0f);
    payload.lowGyroDps10[0] = scaledFloatToI16(imu.gyroXDps, 10.0f);
    payload.lowGyroDps10[1] = scaledFloatToI16(imu.gyroYDps, 10.0f);
    payload.lowGyroDps10[2] = scaledFloatToI16(imu.gyroZDps, 10.0f);
    payload.rollDeg10 = scaledFloatToI16(imu.rollDeg, 10.0f);
    payload.pitchDeg10 = scaledFloatToI16(imu.pitchDeg, 10.0f);
    payload.yawDeg10 = scaledFloatToI16(imu.yawDeg, 10.0f);
    payload.tiltDeg10 = scaledFloatToI16(imu.tiltAngleDeg, 10.0f);
    payload.highGUpdatedMs = highGImuState_.lastUpdatedMs;
    payload.highRaw[0] = highGImuState_.rawX;
    payload.highRaw[1] = highGImuState_.rawY;
    payload.highRaw[2] = highGImuState_.rawZ;
    payload.highAccelMg[0] = scaledFloatToI16(highGImuState_.accelXG, 1000.0f);
    payload.highAccelMg[1] = scaledFloatToI16(highGImuState_.accelYG, 1000.0f);
    payload.highAccelMg[2] = scaledFloatToI16(highGImuState_.accelZG, 1000.0f);
    payload.baroUpdatedMs = baro.lastUpdatedMs;
    payload.pressurePa = scaledFloatToI32(baro.pressurePa, 1.0f);
    payload.rawAltitudeCm = scaledFloatToI32(baro.rawAltitudeM, 100.0f);
    payload.filteredAltitudeCm = scaledFloatToI32(baro.altitudeM, 100.0f);
    payload.batteryMv = telemetryState_.power.batteryMv;

    return enqueueFrame(nura_log::RecordType::FAST_SAMPLE,
                        nowMs,
                        &payload,
                        static_cast<uint16_t>(sizeof(payload)));
}

bool FlightLogTask::enqueueSlowSample(uint32_t nowMs)
{
    const GpsData &gps = gpsState_.data;

    nura_log::SlowSamplePayload payload;
    memset(&payload, 0, sizeof(payload));
    payload.state = static_cast<uint8_t>(flightState_.state);
    payload.gpsFlags = gpsFlags();
    payload.healthFlags = healthFlags();
    payload.magUpdatedMs = magnetometerState_.lastUpdatedMs;
    payload.magRaw[0] = magnetometerState_.rawX;
    payload.magRaw[1] = magnetometerState_.rawY;
    payload.magRaw[2] = magnetometerState_.rawZ;
    payload.magUt10[0] = scaledFloatToI16(magnetometerState_.magXuT, 10.0f);
    payload.magUt10[1] = scaledFloatToI16(magnetometerState_.magYuT, 10.0f);
    payload.magUt10[2] = scaledFloatToI16(magnetometerState_.magZuT, 10.0f);
    payload.gpsUpdatedMs = gps.lastUpdatedMs;
    payload.latitudeE7 = scaledDoubleToI32(gps.latitudeDeg, 10000000.0);
    payload.longitudeE7 = scaledDoubleToI32(gps.longitudeDeg, 10000000.0);
    payload.gpsAltitudeCm = scaledDoubleToI32(gps.altitudeM, 100.0);
    payload.gpsSpeedCms = scaledFloatToI16(static_cast<float>(gps.speedMps), 100.0f);
    payload.gpsCourseDeg10 = scaledFloatToI16(static_cast<float>(gps.courseDeg), 10.0f);
    payload.gpsHdop100 = scaledDoubleToU16(gps.hdop, 100.0);
    payload.gpsSatellites = gps.satellites > 255U ? 255U : static_cast<uint8_t>(gps.satellites);
    payload.gpsCharsProcessed = gps.charsProcessed;
    payload.gpsPassedChecksum = gps.passedChecksum;
    payload.gpsFailedChecksum = gps.failedChecksum;
    payload.baroFaultFlags = telemetryState_.barometer.faultFlags;
    payload.baroConsecutiveReadFailCount = telemetryState_.barometer.consecutiveReadFailCount;
    payload.baroConsecutiveBadValueCount = telemetryState_.barometer.consecutiveBadValueCount;
    payload.baroTotalBadValueCount = telemetryState_.barometer.totalBadValueCount;

    return enqueueFrame(nura_log::RecordType::SLOW_SAMPLE,
                        nowMs,
                        &payload,
                        static_cast<uint16_t>(sizeof(payload)));
}

bool FlightLogTask::enqueueDecision(const FlightDecisionTrace &decision)
{
    nura_log::DecisionPayload payload;
    memset(&payload, 0, sizeof(payload));
    payload.decisionSeq = decision.seq;
    payload.state = static_cast<uint8_t>(decision.state);
    payload.kind = static_cast<uint8_t>(decision.kind);
    payload.result = static_cast<uint8_t>(decision.result);
    payload.count0 = decision.count0;
    payload.count1 = decision.count1;
    payload.reason = decision.reason;
    payload.value0 = decision.value0;
    payload.value1 = decision.value1;
    payload.value2 = decision.value2;
    payload.value3 = decision.value3;

    return enqueueFrame(nura_log::RecordType::DECISION,
                        decision.timestampMs,
                        &payload,
                        static_cast<uint16_t>(sizeof(payload)));
}

bool FlightLogTask::enqueueEvent(nura_log::EventId eventId,
                                 uint32_t nowMs,
                                 State previousState,
                                 State currentState,
                                 uint32_t data0,
                                 uint32_t data1)
{
    nura_log::EventPayload payload;
    memset(&payload, 0, sizeof(payload));
    payload.eventId = static_cast<uint8_t>(eventId);
    payload.previousState = static_cast<uint8_t>(previousState);
    payload.currentState = static_cast<uint8_t>(currentState);
    payload.data0 = data0;
    payload.data1 = data1;

    return enqueueFrame(nura_log::RecordType::EVENT,
                        nowMs,
                        &payload,
                        static_cast<uint16_t>(sizeof(payload)));
}

bool FlightLogTask::enqueueFrame(nura_log::RecordType type,
                                 uint32_t timestampMs,
                                 const void *payload,
                                 uint16_t payloadLength)
{
    const uint32_t sequence = nextSequence_;
    const size_t frameLength = nura_log::encodeFrame(type,
                                                     sequence,
                                                     timestampMs,
                                                     payload,
                                                     payloadLength,
                                                     scratch_,
                                                     sizeof(scratch_));
    if (frameLength == 0U || frameLength > 65535U)
    {
        return false;
    }

    ++nextSequence_;
    return ramBuffer_.push(scratch_, static_cast<uint16_t>(frameLength));
}

void FlightLogTask::drainToStorage(uint32_t nowMs, uint16_t maxRecords)
{
    if (!storageStarted_ || !storage_.healthy())
    {
        telemetryState_.health.storageOk = false;
        return;
    }

    for (uint16_t i = 0U; i < maxRecords; ++i)
    {
        uint16_t length = 0U;
        if (!ramBuffer_.pop(scratch_, static_cast<uint16_t>(sizeof(scratch_)), length))
        {
            break;
        }

        if (!storage_.append(scratch_, length))
        {
            storageStarted_ = false;
            telemetryState_.health.storageOk = false;
            enqueueEvent(nura_log::EventId::STORAGE_FAULT,
                         nowMs,
                         flightState_.state,
                         flightState_.state,
                         length,
                         ramBuffer_.recordCount());
            LOGE(logger_, nowMs, "flight_log", "storage append failed");
            break;
        }
    }
}

uint8_t FlightLogTask::fastFlags() const
{
    uint8_t flags = 0U;
    const ImuData &imu = imuState_.data;
    const BarometerTelemetryData &baro = telemetryState_.barometer;
    if (imu.lastUpdatedMs != 0U)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagLowImuUpdated);
    }
    if (imu.attitudeValid)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagAttitudeValid);
    }
    if (imu.tiltValid)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagTiltValid);
    }
    if (baro.valid)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagBaroValid);
    }
    if (baro.referenceValid)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagBaroReferenceValid);
    }
    if (baro.fault)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagBaroFault);
    }
    if (gpsState_.data.hasFix)
    {
        flags = static_cast<uint8_t>(flags | kFastFlagGpsFix);
    }
    return flags;
}

uint8_t FlightLogTask::gpsFlags() const
{
    uint8_t flags = 0U;
    if (gpsState_.data.lastUpdatedMs != 0U)
    {
        flags = static_cast<uint8_t>(flags | kGpsFlagValid);
    }
    if (gpsState_.data.hasFix)
    {
        flags = static_cast<uint8_t>(flags | kGpsFlagFix);
    }
    return flags;
}

uint16_t FlightLogTask::healthFlags() const
{
    uint16_t flags = 0U;
    if (telemetryState_.health.highAccelOk)
    {
        flags = static_cast<uint16_t>(flags | kHealthHighAccelOk);
    }
    if (telemetryState_.health.magOk)
    {
        flags = static_cast<uint16_t>(flags | kHealthMagOk);
    }
    if (storageStarted_ && storage_.healthy())
    {
        flags = static_cast<uint16_t>(flags | kHealthStorageOk);
    }
    if (telemetryState_.health.pyroContinuityOk)
    {
        flags = static_cast<uint16_t>(flags | kHealthPyroContinuityOk);
    }
    if (telemetryState_.health.deployFired)
    {
        flags = static_cast<uint16_t>(flags | kHealthDeployFired);
    }
    return flags;
}

void FlightLogTask::stopAtGround(uint32_t nowMs)
{
    enqueueEvent(nura_log::EventId::GROUND_STOP,
                 nowMs,
                 flightState_.state,
                 flightState_.state,
                 ramBuffer_.droppedRecords(),
                 ramBuffer_.used());
    drainToStorage(nowMs, 65535U);
    if (storageStarted_)
    {
        storage_.flush();
    }
    storage_.stop();
    telemetryState_.health.storageOk = false;
    stopped_ = true;
    LOGI(logger_, nowMs, "flight_log", "stopped at ground");
}

int16_t FlightLogTask::scaledFloatToI16(float value, float scale)
{
    if (!isfinite(value) || !isfinite(scale))
    {
        return 0;
    }
    const float scaled = value * scale;
    if (scaled > static_cast<float>(kI16Max))
    {
        return kI16Max;
    }
    if (scaled < static_cast<float>(kI16Min))
    {
        return kI16Min;
    }
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int32_t FlightLogTask::scaledFloatToI32(float value, float scale)
{
    if (!isfinite(value) || !isfinite(scale))
    {
        return 0;
    }
    const float scaled = value * scale;
    if (scaled > static_cast<float>(kI32Max))
    {
        return kI32Max;
    }
    if (scaled < static_cast<float>(kI32Min))
    {
        return kI32Min;
    }
    return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int32_t FlightLogTask::scaledDoubleToI32(double value, double scale)
{
    if (!isfinite(value) || !isfinite(scale))
    {
        return 0;
    }
    const double scaled = value * scale;
    if (scaled > static_cast<double>(kI32Max))
    {
        return kI32Max;
    }
    if (scaled < static_cast<double>(kI32Min))
    {
        return kI32Min;
    }
    return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

uint16_t FlightLogTask::scaledDoubleToU16(double value, double scale)
{
    if (!isfinite(value) || !isfinite(scale))
    {
        return 0U;
    }
    const double scaled = value * scale;
    if (scaled <= 0.0)
    {
        return 0U;
    }
    if (scaled > 65535.0)
    {
        return 65535U;
    }
    return static_cast<uint16_t>(scaled + 0.5);
}
