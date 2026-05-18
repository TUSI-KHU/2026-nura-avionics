#include "magnetometer_task.h"

#include "board_pinmap.h"

MagnetometerTask::MagnetometerTask(LIS3MDLHAL &magnetometer,
                                   MagnetometerState &magnetometerState,
                                   TelemetryState &telemetryState,
                                   Logger &logger,
                                   const IAppConfig &config)
    : RecoverableTask(TaskCriticality::NON_CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      magnetometer_(magnetometer),
      magnetometerState_(magnetometerState),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config)
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
        telemetryState_.health.magOk = false;
        markReadFailure();
        return true;
    }

    updateState(sample);
    telemetryState_.health.magOk = true;
    markReadSuccess();
    return true;
}

uint32_t MagnetometerTask::periodMs() const
{
    return config_.imuTaskPeriodMs();
}

bool MagnetometerTask::recover(uint32_t nowMs)
{
    return initialize(nowMs);
}

bool MagnetometerTask::initialize(uint32_t nowMs)
{
    (void)nowMs;
    const bool ok = magnetometer_.begin(BoardPinMap::LIS3MDL::i2cAddress,
                                        BoardPinMap::I2cBus::wire());
    magnetometerState_.connected = ok;
    magnetometerState_.hasNewData = false;
    telemetryState_.health.magOk = false;
    return ok;
}

void MagnetometerTask::resetState(uint32_t nowMs)
{
    magnetometerState_ = MagnetometerState{};
    magnetometerState_.lastUpdatedMs = nowMs;
    telemetryState_.health.magOk = false;
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
