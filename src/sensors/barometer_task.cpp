#include "barometer_task.h"

#include <Wire.h>
#include <math.h>

#include "board_pinmap.h"

namespace
{
    float relativeAltitudeM(float pressurePa, float referencePressurePa)
    {
        if (!isfinite(pressurePa) || !isfinite(referencePressurePa) ||
            pressurePa <= 0.0f || referencePressurePa <= 0.0f)
        {
            return 0.0f;
        }

        return NuraConstants::Atmosphere::kStandardAtmosphereMeters *
               (1.0f - powf(pressurePa / referencePressurePa, NuraConstants::Atmosphere::kPressureExponent));
    }

    float sortedMedian(float values[], uint8_t count)
    {
        for (uint8_t i = 1U; i < count; ++i)
        {
            const float value = values[i];
            uint8_t j = i;
            while (j > 0U && values[j - 1U] > value)
            {
                values[j] = values[j - 1U];
                --j;
            }
            values[j] = value;
        }

        return values[count / 2U];
    }
}

BarometerTask::BarometerTask(MPL3115A2HAL &barometer, TelemetryState &telemetryState, Logger &logger, const IAppConfig &config)
    : barometer_(barometer),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config)
{
}

const char *BarometerTask::name() const
{
    return "baro";
}

bool BarometerTask::init()
{
    clearReading(0U);
    resetHealth();
    initialized_ = initialize(0U);
    return true;
}

bool BarometerTask::tick(uint32_t nowMs)
{
    if (!initialized_)
    {
        if ((nowMs - lastInitAttemptMs_) >= config_.barometerRecoveryIntervalMs())
        {
            initialized_ = initialize(nowMs);
        }
        return true;
    }

    Mpl3115a2Reading sample;
    if (!barometer_.read(sample, nowMs))
    {
        recordReadFailure(nowMs);
        return true;
    }

    BarometerTelemetryData &baro = telemetryState_.barometer;
    consecutiveReadFailCount_ = 0U;
    baro.consecutiveReadFailCount = 0U;
    if (!isfinite(sample.pressurePa) || sample.pressurePa <= 0.0f)
    {
        recordBadValue(nowMs, BARO_FAULT_BAD_VALUE);
        return true;
    }

    if (!baro.referenceValid)
    {
        baro.referencePressurePa = sample.pressurePa;
        baro.referenceValid = true;
    }

    const float rawAltitudeM = relativeAltitudeM(sample.pressurePa, baro.referencePressurePa);
    if (!sampleAltitudeValid(rawAltitudeM))
    {
        recordBadValue(nowMs, BARO_FAULT_BAD_VALUE);
        return true;
    }

    publishValidSample(sample, rawAltitudeM);
    return true;
}

uint32_t BarometerTask::periodMs() const
{
    return config_.barometerTaskPeriodMs();
}

bool BarometerTask::initialize(uint32_t nowMs)
{
    lastInitAttemptMs_ = nowMs;
    const bool ok = barometer_.begin(BoardPinMap::I2cBus::wire());
    if (ok)
    {
        LOGI(logger_, nowMs, "baro", "mpl3115a2 initialized");
    }
    else
    {
        LOGW(logger_, nowMs, "baro", "mpl3115a2 init failed");
    }
    return ok;
}

void BarometerTask::clearReading(uint32_t nowMs)
{
    telemetryState_.barometer.valid = false;
    telemetryState_.barometer.lastUpdatedMs = nowMs;
    lastValidSampleMs_ = 0U;
    altitudeWindowHead_ = 0U;
    altitudeWindowCount_ = 0U;
    filterReady_ = false;
}

void BarometerTask::resetHealth()
{
    consecutiveReadFailCount_ = 0U;
    consecutiveBadValueCount_ = 0U;
    totalBadValueCount_ = 0U;

    BarometerTelemetryData &baro = telemetryState_.barometer;
    baro.fault = false;
    baro.faultFlags = BARO_FAULT_NONE;
    baro.consecutiveReadFailCount = 0U;
    baro.consecutiveBadValueCount = 0U;
    baro.totalBadValueCount = 0U;
}

void BarometerTask::recordReadFailure(uint32_t nowMs)
{
    if (consecutiveReadFailCount_ < 255U)
    {
        ++consecutiveReadFailCount_;
    }

    BarometerTelemetryData &baro = telemetryState_.barometer;
    baro.consecutiveReadFailCount = consecutiveReadFailCount_;

    if (consecutiveReadFailCount_ >= NuraConstants::Sensors::kBarometerConsecutiveReadFailFault)
    {
        markFault(nowMs, BARO_FAULT_READ_FAIL);
    }

    if (lastValidSampleMs_ != 0U &&
        (nowMs - lastValidSampleMs_) >= NuraConstants::Sensors::kBarometerStaleFaultMs)
    {
        markFault(nowMs, BARO_FAULT_STALE);
    }
}

bool BarometerTask::sampleAltitudeValid(float altitudeM) const
{
    return isfinite(altitudeM) &&
           altitudeM >= NuraConstants::Sensors::kBarometerMinAltitudeAglM &&
           altitudeM <= NuraConstants::Sensors::kBarometerMaxAltitudeAglM;
}

void BarometerTask::recordBadValue(uint32_t nowMs, uint16_t faultFlag)
{
    consecutiveReadFailCount_ = 0U;
    if (consecutiveBadValueCount_ < 255U)
    {
        ++consecutiveBadValueCount_;
    }
    if (totalBadValueCount_ < 255U)
    {
        ++totalBadValueCount_;
    }

    BarometerTelemetryData &baro = telemetryState_.barometer;
    baro.consecutiveReadFailCount = 0U;
    baro.consecutiveBadValueCount = consecutiveBadValueCount_;
    baro.totalBadValueCount = totalBadValueCount_;

    if (consecutiveBadValueCount_ >= NuraConstants::Sensors::kBarometerBadValueConsecutiveFault ||
        totalBadValueCount_ >= NuraConstants::Sensors::kBarometerBadValueTotalFault)
    {
        markFault(nowMs, faultFlag);
    }
}

void BarometerTask::publishValidSample(const Mpl3115a2Reading &sample, float rawAltitudeM)
{
    consecutiveReadFailCount_ = 0U;
    consecutiveBadValueCount_ = 0U;

    BarometerTelemetryData &baro = telemetryState_.barometer;
    baro.valid = true;
    baro.consecutiveReadFailCount = 0U;
    baro.consecutiveBadValueCount = 0U;
    baro.pressurePa = sample.pressurePa;
    baro.rawAltitudeM = rawAltitudeM;
    baro.altitudeM = filterAltitude(rawAltitudeM);
    baro.lastUpdatedMs = sample.sampleMs;
    lastValidSampleMs_ = sample.sampleMs;
}

void BarometerTask::markFault(uint32_t nowMs, uint16_t faultFlag)
{
    BarometerTelemetryData &baro = telemetryState_.barometer;
    if ((baro.faultFlags & faultFlag) == 0U)
    {
        LOGW(logger_, nowMs, "baro", "barometer fault");
    }

    baro.fault = true;
    baro.faultFlags = static_cast<uint16_t>(baro.faultFlags | faultFlag);
    baro.valid = false;
    altitudeWindowHead_ = 0U;
    altitudeWindowCount_ = 0U;
    filterReady_ = false;
}

float BarometerTask::filterAltitude(float rawAltitudeM)
{
    altitudeWindowM_[altitudeWindowHead_] = rawAltitudeM;
    altitudeWindowHead_ = static_cast<uint8_t>((altitudeWindowHead_ + 1U) %
                                               NuraConstants::Sensors::kBarometerMedianWindowSamples);
    if (altitudeWindowCount_ < NuraConstants::Sensors::kBarometerMedianWindowSamples)
    {
        ++altitudeWindowCount_;
    }

    float windowCopy[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0U; i < altitudeWindowCount_; ++i)
    {
        windowCopy[i] = altitudeWindowM_[i];
    }

    const float medianAltitudeM = sortedMedian(windowCopy, altitudeWindowCount_);
    if (!filterReady_)
    {
        filteredAltitudeM_ = medianAltitudeM;
        filterReady_ = true;
        return filteredAltitudeM_;
    }

    filteredAltitudeM_ += NuraConstants::Sensors::kBarometerAltitudeLpfAlpha *
                          (medianAltitudeM - filteredAltitudeM_);
    return filteredAltitudeM_;
}
