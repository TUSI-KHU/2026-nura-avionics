#include "barometer_task.h"

#include <Wire.h>
#include <math.h>

#include "board_pinmap.h"

namespace
{
    constexpr float kStandardAtmosphereMeters = 44330.0f;
    constexpr float kPressureExponent = 0.19029495f;
    constexpr float kAltitudeFilterAlpha = 0.35f;
    constexpr uint8_t kAltitudeMedianWindow = 3U;

    float relativeAltitudeM(float pressurePa, float referencePressurePa)
    {
        if (!isfinite(pressurePa) || !isfinite(referencePressurePa) ||
            pressurePa <= 0.0f || referencePressurePa <= 0.0f)
        {
            return 0.0f;
        }

        return kStandardAtmosphereMeters * (1.0f - powf(pressurePa / referencePressurePa, kPressureExponent));
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
        clearReading(nowMs);
        return true;
    }

    BarometerTelemetryData &baro = telemetryState_.barometer;
    baro.valid = true;
    baro.pressurePa = sample.pressurePa;
    if (!baro.referenceValid)
    {
        baro.referencePressurePa = sample.pressurePa;
        baro.referenceValid = true;
    }
    baro.rawAltitudeM = relativeAltitudeM(sample.pressurePa, baro.referencePressurePa);
    baro.altitudeM = filterAltitude(baro.rawAltitudeM);
    baro.lastUpdatedMs = sample.sampleMs;
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
    altitudeWindowHead_ = 0U;
    altitudeWindowCount_ = 0U;
    filterReady_ = false;
}

float BarometerTask::filterAltitude(float rawAltitudeM)
{
    altitudeWindowM_[altitudeWindowHead_] = rawAltitudeM;
    altitudeWindowHead_ = static_cast<uint8_t>((altitudeWindowHead_ + 1U) % kAltitudeMedianWindow);
    if (altitudeWindowCount_ < kAltitudeMedianWindow)
    {
        ++altitudeWindowCount_;
    }

    float windowCopy[kAltitudeMedianWindow] = {0.0f, 0.0f, 0.0f};
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

    filteredAltitudeM_ += kAltitudeFilterAlpha * (medianAltitudeM - filteredAltitudeM_);
    return filteredAltitudeM_;
}
