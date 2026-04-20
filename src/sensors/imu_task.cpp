#include "imu_task.h"

#include <math.h>

IMUTask::IMUTask(MPU6050HAL &imu)
    : RecoverableTask(TaskCriticality::CRITICAL, 3U, 5U, 1000U),
      imu_(imu) {}

const char *IMUTask::name() const
{
    return "imu";
}

bool IMUTask::init(SystemContext &ctx)
{
    // 초기화 전 IMU 컨텍스트를 안전한 기본값으로 리셋한다.
    ctx.imu.accelXMps2 = 0.0f;
    ctx.imu.accelYMps2 = 0.0f;
    ctx.imu.accelZMps2 = 0.0f;
    ctx.imu.gyroXDps = 0.0f;
    ctx.imu.gyroYDps = 0.0f;
    ctx.imu.gyroZDps = 0.0f;
    ctx.imu.lastUpdatedMs = 0U;

    const bool ok = imu_.begin(0x69);

    if (!ok)
    {
        // 초기화 실패는 logger task가 돌기 전일 수 있어 부팅 단계에서 유실될 수 있다.
        LOGE(ctx.logger, 0U, "imu", "mpu6050 init failed");

        return false;
    }

    markInitialized();
    LOGI(ctx.logger, 0U, "imu", "mpu6050 initialized");

    return true;
}

bool IMUTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    // 센서 태스크는 읽기 성공/실패 관측만 기록하고 health 전이는 watchdog에 맡긴다.
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
    return 10U;
}

bool IMUTask::recover(uint32_t nowMs)
{
    (void)nowMs;

    // 센서 begin을 다시 시도
    const bool ok = imu_.begin(0x69);

    return ok;
}
