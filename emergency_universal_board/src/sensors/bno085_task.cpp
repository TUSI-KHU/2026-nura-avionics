#include "sensors/bno085_task.h"

#include <Arduino.h>
#include <math.h>

#include "board_pinmap.h"
#include "nura_constants.h"

BNO085Task::BNO085Task(BNO085HAL &imu,
                       ImuState &imuState,
                       HighGImuState &highGImuState,
                       TelemetryState &telemetryState,
                       Logger &logger,
                       const IAppConfig &config)
    : RecoverableTask(TaskCriticality::CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      imu_(imu),
      imuState_(imuState),
      highGImuState_(highGImuState),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config)
{
}

const char *BNO085Task::name() const
{
    return "bno085";
}

bool BNO085Task::init()
{
    imuState_.data = ImuData{};
    highGImuState_ = HighGImuState{};
    telemetryState_.health.highAccelOk = false;

    if (!initializeDevice(0U))
    {
        LOGE(logger_, 0U, "bno085", "bno085 init failed");
        return false;
    }

    markInitialized();
    LOGI(logger_, 0U, "bno085", "bno085 initialized");
    return true;
}

bool BNO085Task::tick(uint32_t nowMs)
{
    Bno085Reading sample;
    if (imu_.read(sample, nowMs))
    {
        publishSample(sample);
        logSample(nowMs);
        markReadSuccess();
    }
    else
    {
        markReadFailure();
    }

    return true;
}

uint32_t BNO085Task::periodMs() const
{
    return config_.imuTaskPeriodMs();
}

bool BNO085Task::recover(uint32_t nowMs)
{
    (void)nowMs;
    return imu_.begin(BoardPinMap::BNO085::wire(), BoardPinMap::BNO085::i2cAddress);
}

bool BNO085Task::initializeDevice(uint32_t logTs)
{
    (void)logTs;

    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        if (imu_.begin(BoardPinMap::BNO085::wire(), BoardPinMap::BNO085::i2cAddress))
        {
            return true;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }

    return false;
}

void BNO085Task::publishSample(const Bno085Reading &sample)
{
    ImuData imuData{};
    imuData.accelXMps2 = sample.accelXMps2;
    imuData.accelYMps2 = sample.accelYMps2;
    imuData.accelZMps2 = sample.accelZMps2;
    imuData.gyroXDps = sample.gyroXDps;
    imuData.gyroYDps = sample.gyroYDps;
    imuData.gyroZDps = sample.gyroZDps;
    imuData.attitudeValid = sample.attitudeValid;
    imuData.rollDeg = sample.rollDeg;
    imuData.pitchDeg = sample.pitchDeg;
    imuData.yawDeg = sample.yawDeg;
    imuData.tiltValid = sample.attitudeValid;
    imuData.tiltAngleDeg = sample.attitudeValid ? sqrtf((sample.rollDeg * sample.rollDeg) + (sample.pitchDeg * sample.pitchDeg)) : 0.0f;
    imuData.lastUpdatedMs = sample.sampleMs;
    imuState_.set_data(imuData);

    highGImuState_.accelXMps2 = sample.accelXMps2;
    highGImuState_.accelYMps2 = sample.accelYMps2;
    highGImuState_.accelZMps2 = sample.accelZMps2;
    highGImuState_.accelXG = sample.accelXMps2 / NuraConstants::Physics::kGravityMps2;
    highGImuState_.accelYG = sample.accelYMps2 / NuraConstants::Physics::kGravityMps2;
    highGImuState_.accelZG = sample.accelZMps2 / NuraConstants::Physics::kGravityMps2;
    highGImuState_.connected = true;
    highGImuState_.hasNewData = true;
    highGImuState_.lastUpdatedMs = sample.sampleMs;
    telemetryState_.health.highAccelOk = true;
}

void BNO085Task::logSample(uint32_t nowMs)
{
    if ((nowMs - lastSampleLogMs_) < NuraConstants::Sensors::kLowGSampleLogIntervalMs)
    {
        return;
    }

    LOGD(logger_, nowMs, "bno085", "sample");
    lastSampleLogMs_ = nowMs;
}
