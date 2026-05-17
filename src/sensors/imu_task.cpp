#include "imu_task.h"

namespace
{
    constexpr uint32_t kSampleLogIntervalMs = 1000U;
}

IMUTask::IMUTask(LSM6DSO32HAL &imu, ImuState &imuState, Logger &logger, const IAppConfig &config)
    : RecoverableTask(TaskCriticality::CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      imu_(imu),
      imuState_(imuState),
      logger_(logger),
      config_(config) {}

const char *IMUTask::name() const
{
    return "imu";
}

bool IMUTask::init()
{
    resetState();

    if (!initializeDevice(0U))
    {
        LOGE(logger_, 0U, "imu", "lsm6dso32 init failed");
        return false;
    }

    markInitialized();
    LOGI(logger_, 0U, "imu", "lsm6dso32 initialized");

    return true;
}

bool IMUTask::tick(uint32_t nowMs)
{
    // 센서 태스크는 읽기 성공/실패 관측만 기록하고 health 전이는 watchdog에 맡긴다.
    Lsm6dso32Reading sample;
    const bool readOk = imu_.read(sample, nowMs);

    if (readOk)
    {
        updateState(sample);
        logSample(nowMs);
        markReadSuccess();
    }
    else
    {
        imuState_.data.connected = false;
        imuState_.data.hasNewData = false;
        markReadFailure();
    }

    return true;
}

uint32_t IMUTask::periodMs() const
{
    return config_.imuTaskPeriodMs();
}

bool IMUTask::recover(uint32_t nowMs)
{
    // 센서 begin을 다시 시도
    return initializeDevice(nowMs);
}

bool IMUTask::initializeDevice(uint32_t logTs)
{
    const bool ok = imu_.begin();
    imuState_.data.connected = ok;
    imuState_.data.hasNewData = false;

    if (!ok)
    {
        LOGW(logger_, logTs, "imu", "lsm6dso32 begin failed");
    }

    return ok;
}

void IMUTask::resetState()
{
    imuState_.set_data(ImuData{});
    lastSampleLogMs_ = 0U;
}

void IMUTask::updateState(const Lsm6dso32Reading &sample)
{
    ImuData data;
    data.accelXMps2 = sample.accelXMps2;
    data.accelYMps2 = sample.accelYMps2;
    data.accelZMps2 = sample.accelZMps2;
    data.gyroXDps = sample.gyroXDps;
    data.gyroYDps = sample.gyroYDps;
    data.gyroZDps = sample.gyroZDps;
    data.temperatureC = sample.temperatureC;
    data.rawAccelX = sample.rawAccelX;
    data.rawAccelY = sample.rawAccelY;
    data.rawAccelZ = sample.rawAccelZ;
    data.rawGyroX = sample.rawGyroX;
    data.rawGyroY = sample.rawGyroY;
    data.rawGyroZ = sample.rawGyroZ;
    data.connected = true;
    data.hasNewData = true;
    data.lastUpdatedMs = sample.sampleMs;

    imuState_.set_data(data);
}

void IMUTask::logSample(uint32_t nowMs)
{
    if ((nowMs - lastSampleLogMs_) < kSampleLogIntervalMs)
    {
        return;
    }

    lastSampleLogMs_ = nowMs;

    if (!Serial)
    {
        return;
    }

    const ImuData &data = imuState_.data;
    Serial.print("[");
    Serial.print(nowMs);
    Serial.print("] low_g_imu raw_accel=");
    Serial.print(data.rawAccelX);
    Serial.print(",");
    Serial.print(data.rawAccelY);
    Serial.print(",");
    Serial.print(data.rawAccelZ);
    Serial.print(" accel_mps2=");
    Serial.print(data.accelXMps2, 3);
    Serial.print(",");
    Serial.print(data.accelYMps2, 3);
    Serial.print(",");
    Serial.print(data.accelZMps2, 3);
    Serial.print(" gyro_dps=");
    Serial.print(data.gyroXDps, 3);
    Serial.print(",");
    Serial.print(data.gyroYDps, 3);
    Serial.print(",");
    Serial.print(data.gyroZDps, 3);
    Serial.print(" temp_c=");
    Serial.print(data.temperatureC, 2);
    Serial.print(" connected=");
    Serial.println(data.connected ? "true" : "false");
}
