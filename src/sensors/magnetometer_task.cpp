#include "magnetometer_task.h"

#include <Arduino.h>

#include "board_pinmap.h"
#include "nura_constants.h"

MagnetometerTask::MagnetometerTask(IMagnetometer &magnetometer,
                                   MagnetometerState &magnetometerState,
                                   SystemHealthState &healthState,
                                   Logger &logger,
                                   const IAppConfig &config,
                                   uint8_t i2cAddress,
                                   TwoWire &wire)
    : RecoverableTask(TaskCriticality::NON_CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      magnetometer_(magnetometer),
      magnetometerState_(magnetometerState),
      healthState_(healthState),
      logger_(logger),
      config_(config),
      i2cAddress_(i2cAddress),
      wire_(wire)
{
}

const char *MagnetometerTask::name() const
{
    return "mag";
}

bool MagnetometerTask::init()
{
    resetState(0U);

    if (!initialize(0U))
    {
        markInitialized();
        markReadFailure();
        LOGW(logger_, 0U, "mag", "lis3mdl init failed");
        return true;
    }

    markInitialized();
    LOGI(logger_, 0U, "mag", "lis3mdl initialized");
    return true;
}

bool MagnetometerTask::tick(uint32_t nowMs)
{
    Lis3mdlReading sample;
    if (!magnetometer_.read(sample, nowMs))
    {
        magnetometerState_.connected = false;
        magnetometerState_.hasNewData = false;
        healthState_.magOk = false;
        markReadFailure();
        return true;
    }

    updateState(sample);
    healthState_.magOk = true;
    markReadSuccess();
    return true;
}

uint32_t MagnetometerTask::periodMs() const
{
    return config_.magnetometerTaskPeriodMs();
}

bool MagnetometerTask::recover(uint32_t nowMs)
{
    (void)nowMs;
    const bool ok = magnetometer_.beginDefault(i2cAddress_, wire_);
    magnetometerState_.connected = ok;
    magnetometerState_.hasNewData = false;
    healthState_.magOk = false;
    return ok;
}

bool MagnetometerTask::initialize(uint32_t nowMs)
{
    (void)nowMs;
    bool ok = false;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        ok = magnetometer_.beginDefault(i2cAddress_, wire_);
        if (ok)
        {
            break;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }

    magnetometerState_.connected = ok;
    magnetometerState_.hasNewData = false;
    healthState_.magOk = false;
    return ok;
}

void MagnetometerTask::resetState(uint32_t nowMs)
{
    magnetometerState_ = MagnetometerState{};
    magnetometerState_.lastUpdatedMs = nowMs;
    healthState_.magOk = false;
}

void MagnetometerTask::updateState(const Lis3mdlReading &sample)
{
    magnetometerState_.rawX = sample.rawX;
    magnetometerState_.rawY = sample.rawY;
    magnetometerState_.rawZ = sample.rawZ;
    magnetometerState_.magXuT = sample.magXuT;
    magnetometerState_.magYuT = sample.magYuT;
    magnetometerState_.magZuT = sample.magZuT;
    magnetometerState_.connected = true;
    magnetometerState_.hasNewData = true;
    magnetometerState_.lastUpdatedMs = sample.sampleMs;
}
