#include "imu_task.h"

#include <math.h>

IMUTask::IMUTask(MPU6050HAL &imu)
    : RecoverableDevice(3U, 100U, 1000U),
      imu_(imu) {}

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
    ctx.health.imuOk = false;
    ctx.health.imuDevice = DeviceHealthInfo();

    const bool ok = imu_.begin(0x69);

    if (!ok)
    {
        LOGE(ctx.logger, 0U, "imu", "mpu6050 init failed");
        ctx.health.imuDevice.healthState = DeviceHealth::FAILED;
        ctx.health.imuOk = false;
        return false;
    }

    markInitialized(ctx, 0U);
    ctx.health.imuOk = true;
    LOGI(ctx.logger, 0U, "imu", "mpu6050 initialized");

    return true;
}

bool IMUTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    Mpu6050Reading sample;
    const bool readOk = imu_.read(sample, nowMs);

    if (readOk)
    {
        ctx.imu.accelXMps2 = sample.accelXMps2;
        ctx.imu.accelYMps2 = sample.accelYMps2;
        ctx.imu.accelZMps2 = sample.accelZMps2;
        ctx.imu.gyroXDps = sample.gyroXDps;
        ctx.imu.gyroYDps = sample.gyroYDps;
        ctx.imu.gyroZDps = sample.gyroZDps;
        ctx.imu.lastUpdatedMs = sample.sampleMs;

        markReadSuccess(ctx, sample.sampleMs);
    }
    else
    {
        markReadFailure(ctx, nowMs);
    }

    return true;
}

uint32_t IMUTask::periodMs() const
{
    return 10U;
}

const char *IMUTask::deviceName() const
{
    return "imu";
}

DeviceHealthInfo &IMUTask::health(SystemContext &ctx) const
{
    return ctx.health.imuDevice;
}

bool IMUTask::recover(SystemContext &ctx, uint32_t nowMs)
{
    (void)ctx;
    (void)nowMs;

    const bool ok = imu_.begin();

    return ok;
}
