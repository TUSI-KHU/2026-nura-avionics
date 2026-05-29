#include "high_g_imu_task.h"

#include <Arduino.h>

#include "nura_constants.h"

HighGImuTask::HighGImuTask(H3LIS331DLHAL &imu,
                           HighGImuState &imuState,
                           TelemetryState &telemetryState,
                           Logger &logger,
                           const IAppConfig &config,
                           uint8_t csPin,
                           H3LIS331DLRange range)
    : RecoverableTask(TaskCriticality::NON_CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      imu_(imu),
      imuState_(imuState),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config),
      csPin_(csPin),
      range_(range) {}

const char *HighGImuTask::name() const
{
    return "high_g_imu";
}

bool HighGImuTask::init()
{
    resetState();

    if (!initializeDevice(0U))
    {
        markInitialized();
        markReadFailure();
        LOGW(logger_, 0U, "high_g_imu", "h3lis331dl init failed");
        return true;
    }

    markInitialized();
    LOGI(logger_, 0U, "high_g_imu", "h3lis331dl initialized");

    return true;
}

bool HighGImuTask::tick(uint32_t nowMs)
{
    H3LIS331DLReading sample;
    const bool readOk = imu_.read(sample, nowMs);

    if (readOk)
    {
        updateState(sample);
        logSample(nowMs);
        markReadSuccess();
    }
    else
    {
        imuState_.connected = false;
        imuState_.hasNewData = false;
        telemetryState_.health.highAccelOk = false;
        markReadFailure();
    }

    return true;
}

uint32_t HighGImuTask::periodMs() const
{
    return config_.imuTaskPeriodMs();
}

bool HighGImuTask::recover(uint32_t nowMs)
{
    return initializeDevice(nowMs);
}

bool HighGImuTask::initializeDevice(uint32_t logTs)
{
    bool ok = false;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        ok = imu_.begin(csPin_, SPI, range_);
        if (ok)
        {
            break;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }

    imuState_.whoAmI = imu_.readWhoAmI();
    imuState_.connected = ok;
    imuState_.hasNewData = false;
    telemetryState_.health.highAccelOk = false;

    if (!ok)
    {
        LOGW(logger_, logTs, "high_g_imu", "h3lis331dl begin failed");
    }

    return ok;
}

void HighGImuTask::resetState()
{
    imuState_.rawX = 0;
    imuState_.rawY = 0;
    imuState_.rawZ = 0;
    imuState_.accelXG = 0.0f;
    imuState_.accelYG = 0.0f;
    imuState_.accelZG = 0.0f;
    imuState_.accelXMps2 = 0.0f;
    imuState_.accelYMps2 = 0.0f;
    imuState_.accelZMps2 = 0.0f;
    imuState_.whoAmI = 0U;
    imuState_.connected = false;
    imuState_.hasNewData = false;
    imuState_.lastUpdatedMs = 0U;
    telemetryState_.health.highAccelOk = false;
    lastSampleLogMs_ = 0U;
}

void HighGImuTask::updateState(const H3LIS331DLReading &sample)
{
    imuState_.rawX = sample.rawX;
    imuState_.rawY = sample.rawY;
    imuState_.rawZ = sample.rawZ;
    imuState_.accelXG = sample.accelXG;
    imuState_.accelYG = sample.accelYG;
    imuState_.accelZG = sample.accelZG;
    imuState_.accelXMps2 = sample.accelXMps2;
    imuState_.accelYMps2 = sample.accelYMps2;
    imuState_.accelZMps2 = sample.accelZMps2;
    imuState_.whoAmI = sample.whoAmI;
    imuState_.connected = true;
    imuState_.hasNewData = true;
    imuState_.lastUpdatedMs = sample.sampleMs;
    telemetryState_.health.highAccelOk = true;
}

void HighGImuTask::logSample(uint32_t nowMs)
{
    if ((nowMs - lastSampleLogMs_) < NuraConstants::Sensors::kHighGSampleLogIntervalMs)
    {
        return;
    }

    lastSampleLogMs_ = nowMs;

    if (!Serial)
    {
        return;
    }

    Serial.print("[");
    Serial.print(nowMs);
    Serial.print("] high_g_imu raw=");
    Serial.print(imuState_.rawX);
    Serial.print(",");
    Serial.print(imuState_.rawY);
    Serial.print(",");
    Serial.print(imuState_.rawZ);
    Serial.print(" g=");
    Serial.print(imuState_.accelXG, 3);
    Serial.print(",");
    Serial.print(imuState_.accelYG, 3);
    Serial.print(",");
    Serial.print(imuState_.accelZG, 3);
    Serial.print(" mps2=");
    Serial.print(imuState_.accelXMps2, 3);
    Serial.print(",");
    Serial.print(imuState_.accelYMps2, 3);
    Serial.print(",");
    Serial.print(imuState_.accelZMps2, 3);
    Serial.print(" who=0x");
    if (imuState_.whoAmI < 0x10U)
    {
        Serial.print("0");
    }
    Serial.print(imuState_.whoAmI, HEX);
    Serial.print(" connected=");
    Serial.println(imuState_.connected ? "true" : "false");
}
