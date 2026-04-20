#include "imu_task.h"

#include <math.h>

namespace
{
    constexpr uint8_t kAlternateImuI2cAddress = 0x69U;
}

IMUTask::IMUTask(MPU6050HAL &imu, ImuState &imuState, Logger &logger, const IAppConfig &config)
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
    // 초기화 전 IMU 컨텍스트를 안전한 기본값으로 리셋한다.
    imuState_.data.accelXMps2 = 0.0f;
    imuState_.data.accelYMps2 = 0.0f;
    imuState_.data.accelZMps2 = 0.0f;
    imuState_.data.gyroXDps = 0.0f;
    imuState_.data.gyroYDps = 0.0f;
    imuState_.data.gyroZDps = 0.0f;
    imuState_.data.lastUpdatedMs = 0U;

    if (!initializeDevice(0U))
    {
        LOGE(logger_, 0U, "imu", "mpu6050 init failed");
        return false;
    }

    markInitialized();
    LOGI(logger_, 0U, "imu", "mpu6050 initialized");

    return true;
}

bool IMUTask::tick(uint32_t nowMs)
{
    // 센서 태스크는 읽기 성공/실패 관측만 기록하고 health 전이는 watchdog에 맡긴다.
    Mpu6050Reading sample;
    const bool readOk = imu_.read(sample, nowMs);

    if (readOk)
    {
        imuState_.data.accelXMps2 = sample.accelXMps2;
        imuState_.data.accelYMps2 = sample.accelYMps2;
        imuState_.data.accelZMps2 = sample.accelZMps2;
        imuState_.data.gyroXDps = sample.gyroXDps;
        imuState_.data.gyroYDps = sample.gyroYDps;
        imuState_.data.gyroZDps = sample.gyroZDps;
        imuState_.data.lastUpdatedMs = sample.sampleMs;

        markReadSuccess();
    }
    else
    {
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
    const uint8_t configuredAddress = config_.imuI2cAddress();
    if (imu_.begin(configuredAddress))
    {
        return true;
    }

    if (configuredAddress == kAlternateImuI2cAddress)
    {
        return false;
    }

    if (!imu_.begin(kAlternateImuI2cAddress))
    {
        return false;
    }

    LOGW(logger_, logTs, "imu", "mpu6050 using fallback address");
    return true;
}
