#include "sensors/mag_task.h"

namespace
{
    constexpr uint8_t kAlternateMagI2cAddress = 0x1CU;
}

MagTask::MagTask(LIS3MDLHAL &mag, MagState &magState, Logger &logger, const IAppConfig &config)
    : RecoverableTask(TaskCriticality::NON_CRITICAL,
                      config.magReadFailureThreshold(),
                      config.magMaxRecoveryAttempts(),
                      config.magRecoveryIntervalMs()),
      mag_(mag),
      magState_(magState),
      logger_(logger),
      config_(config) {}

const char *MagTask::name() const
{
    return "mag";
}

bool MagTask::init()
{
    magState_.data.magXuT = 0.0f;
    magState_.data.magYuT = 0.0f;
    magState_.data.magZuT = 0.0f;
    magState_.data.magXGauss = 0.0f;
    magState_.data.magYGauss = 0.0f;
    magState_.data.magZGauss = 0.0f;
    magState_.data.rawX = 0;
    magState_.data.rawY = 0;
    magState_.data.rawZ = 0;
    magState_.data.lastUpdatedMs = 0U;

    if (!initializeDevice(0U))
    {
        LOGE(logger_, 0U, "mag", "lis3mdl init failed");
        return false;
    }

    markInitialized();
    LOGI(logger_, 0U, "mag", "lis3mdl initialized");

    return true;
}

bool MagTask::tick(uint32_t nowMs)
{
    Lis3mdlReading sample;
    const bool readOk = mag_.read(sample, nowMs);

    if (readOk)
    {
        magState_.data.magXuT = sample.magXuT;
        magState_.data.magYuT = sample.magYuT;
        magState_.data.magZuT = sample.magZuT;
        magState_.data.magXGauss = sample.magXGauss;
        magState_.data.magYGauss = sample.magYGauss;
        magState_.data.magZGauss = sample.magZGauss;
        magState_.data.rawX = sample.rawX;
        magState_.data.rawY = sample.rawY;
        magState_.data.rawZ = sample.rawZ;
        magState_.data.lastUpdatedMs = sample.sampleMs;

        markReadSuccess();
    }
    else
    {
        markReadFailure();
    }

    return true;
}

uint32_t MagTask::periodMs() const
{
    return config_.magTaskPeriodMs();
}

bool MagTask::recover(uint32_t nowMs)
{
    return initializeDevice(nowMs);
}

bool MagTask::initializeDevice(uint32_t logTs)
{
    const uint8_t configuredAddress = config_.magI2cAddress();
    if (mag_.begin(configuredAddress))
    {
        return true;
    }

    if (configuredAddress == kAlternateMagI2cAddress)
    {
        return false;
    }

    if (!mag_.begin(kAlternateMagI2cAddress))
    {
        return false;
    }

    LOGW(logger_, logTs, "mag", "lis3mdl using fallback address");
    return true;
}
