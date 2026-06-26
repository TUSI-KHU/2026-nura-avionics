#include "high_g_imu_task.h"

#include <Arduino.h>
#include <math.h>

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
    const bool ok = imu_.begin(csPin_, SPI, range_);
    imuState_.whoAmI = imu_.readWhoAmI();
    imuState_.connected = ok;
    imuState_.hasNewData = false;
    telemetryState_.health.highAccelOk = false;
    if (!ok)
    {
        LOGW(logger_, nowMs, "high_g_imu", "h3lis331dl begin failed");
        calibrationValid_ = false;
        filterReady_ = false;
        return false;
    }

    calibrationValid_ = false;
    filterReady_ = false;
    return true;
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
        calibrationValid_ = false;
        filterReady_ = false;
        return false;
    }

    if (!calibrateStationary(logTs))
    {
        LOGW(logger_, logTs, "high_g_imu", "h3lis331dl calibration skipped");
    }

    return true;
}

bool HighGImuTask::calibrateStationary(uint32_t logTs)
{
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    uint8_t samples = 0U;

    for (uint8_t i = 0U; i < NuraConstants::H3LIS331DL::kCalibrationSamples; ++i)
    {
        H3LIS331DLReading sample;
        if (imu_.read(sample, millis()))
        {
            sumX += sample.accelXG;
            sumY += sample.accelYG;
            sumZ += sample.accelZG;
            ++samples;
        }
        delay(NuraConstants::H3LIS331DL::kCalibrationSampleDelayMs);
    }

    if (samples == 0U)
    {
        calibrationValid_ = false;
        filterReady_ = false;
        return false;
    }

    const float avgX = sumX / static_cast<float>(samples);
    const float avgY = sumY / static_cast<float>(samples);
    const float avgZ = sumZ / static_cast<float>(samples);
    const float norm = sqrtf((avgX * avgX) + (avgY * avgY) + (avgZ * avgZ));
    if (!isfinite(norm) ||
        norm < NuraConstants::H3LIS331DL::kCalibrationMinNormG ||
        norm > NuraConstants::H3LIS331DL::kCalibrationMaxNormG)
    {
        calibrationValid_ = false;
        filterReady_ = false;
        return false;
    }

    const float invNorm = 1.0f / norm;
    offsetXG_ = avgX - (avgX * invNorm);
    offsetYG_ = avgY - (avgY * invNorm);
    offsetZG_ = avgZ - (avgZ * invNorm);
    filteredXG_ = avgX - offsetXG_;
    filteredYG_ = avgY - offsetYG_;
    filteredZG_ = avgZ - offsetZG_;
    calibrationValid_ = true;
    filterReady_ = true;

    LOGI(logger_, logTs, "high_g_imu", "h3lis331dl calibrated");
    return true;
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
    offsetXG_ = 0.0f;
    offsetYG_ = 0.0f;
    offsetZG_ = 0.0f;
    filteredXG_ = 0.0f;
    filteredYG_ = 0.0f;
    filteredZG_ = 0.0f;
    calibrationValid_ = false;
    filterReady_ = false;
}

void HighGImuTask::updateState(const H3LIS331DLReading &sample)
{
    float xG = 0.0f;
    float yG = 0.0f;
    float zG = 0.0f;
    applyCalibrationAndFilter(sample, xG, yG, zG);

    imuState_.rawX = sample.rawX;
    imuState_.rawY = sample.rawY;
    imuState_.rawZ = sample.rawZ;
    imuState_.accelXG = xG;
    imuState_.accelYG = yG;
    imuState_.accelZG = zG;
    imuState_.accelXMps2 = xG * NuraConstants::Physics::kGravityMps2;
    imuState_.accelYMps2 = yG * NuraConstants::Physics::kGravityMps2;
    imuState_.accelZMps2 = zG * NuraConstants::Physics::kGravityMps2;
    imuState_.whoAmI = sample.whoAmI;
    imuState_.connected = true;
    imuState_.hasNewData = true;
    imuState_.lastUpdatedMs = sample.sampleMs;
    telemetryState_.health.highAccelOk = true;
}

void HighGImuTask::applyCalibrationAndFilter(const H3LIS331DLReading &sample,
                                             float &xG,
                                             float &yG,
                                             float &zG)
{
    xG = sample.accelXG;
    yG = sample.accelYG;
    zG = sample.accelZG;

    if (calibrationValid_)
    {
        xG -= offsetXG_;
        yG -= offsetYG_;
        zG -= offsetZG_;
    }

    if (!filterReady_)
    {
        filteredXG_ = xG;
        filteredYG_ = yG;
        filteredZG_ = zG;
        filterReady_ = true;
    }
    else
    {
        const float alpha = NuraConstants::H3LIS331DL::kFilterAlpha;
        filteredXG_ += alpha * (xG - filteredXG_);
        filteredYG_ += alpha * (yG - filteredYG_);
        filteredZG_ += alpha * (zG - filteredZG_);
    }

    xG = filteredXG_;
    yG = filteredYG_;
    zG = filteredZG_;
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
    Serial.print(imuState_.connected ? "true" : "false");
    Serial.print(" calibrated=");
    Serial.println(calibrationValid_ ? "true" : "false");
}
