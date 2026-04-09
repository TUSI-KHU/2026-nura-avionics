#include "imu_task.h"

#include <math.h>

namespace
{
constexpr float kMaxAccelAbsMps2 = 90.0f;
constexpr float kMaxGyroAbsDps = 550.0f;
}

IMUTask::IMUTask(MPU6050HAL &imu)
    : imu_(imu) {}

const char *IMUTask::name() const
{
    return "imu";
}

bool IMUTask::init(SystemContext &ctx)
{
    ctx.imu.accelXMps2 = 0.0f;
    ctx.imu.accelYMps2 = 0.0f;
    ctx.imu.accelZMps2 = 0.0f;
    ctx.imu.gyroXDps = 0.0f;
    ctx.imu.gyroYDps = 0.0f;
    ctx.imu.gyroZDps = 0.0f;
    ctx.imu.lastUpdatedMs = 0U;

    const bool ok = imu_.begin();
    ctx.health.imuOk = ok;
    nextReconnectAttemptMs_ = 0U;

    if (ok)
    {
        LOGI(ctx.logger, 0U, "imu", "mpu6050 initialized");
    }
    else
    {
        LOGE(ctx.logger, 0U, "imu", "mpu6050 init failed");
    }

    return true;
}

bool IMUTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    const bool previousImuOk = ctx.health.imuOk;

    Mpu6050Reading sample;
    bool readOk = imu_.read(sample, nowMs);

    if (!readOk && (int32_t)(nowMs - nextReconnectAttemptMs_) >= 0)
    {
        nextReconnectAttemptMs_ = nowMs + kReconnectIntervalMs;
        if (imu_.begin())
        {
            readOk = imu_.read(sample, nowMs);
        }
    }

    const bool sampleOk = readOk && isSampleValid(sample);

    if (sampleOk)
    {
        ctx.imu.accelXMps2 = sample.accelXMps2;
        ctx.imu.accelYMps2 = sample.accelYMps2;
        ctx.imu.accelZMps2 = sample.accelZMps2;
        ctx.imu.gyroXDps = sample.gyroXDps;
        ctx.imu.gyroYDps = sample.gyroYDps;
        ctx.imu.gyroZDps = sample.gyroZDps;
        ctx.imu.lastUpdatedMs = sample.sampleMs;
    }

    const bool imuHealthy = readOk && sampleOk;
    ctx.health.imuOk = imuHealthy;

    if (imuHealthy != previousImuOk)
    {
        if (imuHealthy)
        {
            LOGW(ctx.logger, nowMs, "imu", "recovered");
        }
        else if (!readOk)
        {
            LOGW(ctx.logger, nowMs, "imu", "i2c read failed");
        }
        else
        {
            LOGW(ctx.logger, nowMs, "imu", "invalid");
        }
    }

    return true;
}

uint32_t IMUTask::periodMs() const
{
    return 10U;
}

bool IMUTask::isSampleValid(const Mpu6050Reading &sample) const
{
    if (!isfinite(sample.accelXMps2) || !isfinite(sample.accelYMps2) || !isfinite(sample.accelZMps2))
    {
        return false;
    }

    if (!isfinite(sample.gyroXDps) || !isfinite(sample.gyroYDps) || !isfinite(sample.gyroZDps))
    {
        return false;
    }

    if (fabsf(sample.accelXMps2) > kMaxAccelAbsMps2 || fabsf(sample.accelYMps2) > kMaxAccelAbsMps2 || fabsf(sample.accelZMps2) > kMaxAccelAbsMps2)
    {
        return false;
    }

    if (fabsf(sample.gyroXDps) > kMaxGyroAbsDps || fabsf(sample.gyroYDps) > kMaxGyroAbsDps || fabsf(sample.gyroZDps) > kMaxGyroAbsDps)
    {
        return false;
    }

    return true;
}
