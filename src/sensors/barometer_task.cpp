#include "barometer_task.h"

#include <Wire.h>

#include "board_pinmap.h"

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
}
