#include "imu_task.h"

IMUTask::IMUTask(MPU6050HAL &imu)
    : imu_(imu) {}

const char *IMUTask::name() const
{
    return "imu";
}

bool IMUTask::init(SystemContext &ctx)
{
    ctx.imu.accelZMps2 = 0.0f;
    ctx.imu.gyroZDps = 0.0f;
    ctx.imu.lastUpdatedMs = 0U;

    const bool ok = imu_.begin();
    ctx.health.imuOk = ok;
    lastReadOk_ = ok;

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
    float accelZ = 0.0f;
    float gyroZ = 0.0f;

    const bool ok = imu_.readZ(accelZ, gyroZ);
    ctx.health.imuOk = ok;

    if (ok)
    {
        ctx.imu.accelZMps2 = accelZ;
        ctx.imu.gyroZDps = gyroZ;
        ctx.imu.lastUpdatedMs = nowMs;
    }

    if (ok != lastReadOk_)
    {
        if (ok)
        {
            LOGW(ctx.logger, nowMs, "imu", "mpu6050 recovered");
        }
        else
        {
            LOGW(ctx.logger, nowMs, "imu", "mpu6050 read failed");
        }
        lastReadOk_ = ok;
    }

    return true;
}

uint32_t IMUTask::periodMs() const
{
    return 10;
}
