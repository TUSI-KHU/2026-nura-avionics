#include "core/recoverable_device/recoverable_device.h"

RecoverableDevice::RecoverableDevice(uint8_t readFailureThreshold,
                                     uint32_t recoveryIntervalMs,
                                     uint32_t maxRetryBeforeFailureMs)
    : readFailureThreshold_(readFailureThreshold),
      recoveryIntervalMs_(recoveryIntervalMs),
      maxRetryBeforeFailureMs_(maxRetryBeforeFailureMs) {}

void RecoverableDevice::markInitialized(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);
    info.healthState = DeviceHealth::NORMAL;
    info.readFails = 0;
    info.lastOkMs = nowMs;
    info.lastRecoveryAttemptMs = 0;
}

void RecoverableDevice::markReadSuccess(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);
    info.healthState = DeviceHealth::NORMAL;
    info.readFails = 0;
    info.lastOkMs = nowMs;
    info.lastRecoveryAttemptMs = 0;
}

void RecoverableDevice::markReadFailure(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);

    if (info.readFails < 255)
    {
        ++info.readFails;
    }

    info.healthState = DeviceHealth::DEGRADED;

    if (info.readFails < readFailureThreshold_)
    {
        return;
    }

    if (hasExceededRecoveryBudget(info, nowMs))
    {
        info.healthState = DeviceHealth::FAILED;
    }
}

bool RecoverableDevice::shouldRecover(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);

    if (info.healthState != DeviceHealth::DEGRADED)
    {
        return false;
    }

    if (info.readFails < readFailureThreshold_)
    {
        return false;
    }

    if (hasExceededRecoveryBudget(info, nowMs))
    {
        info.healthState = DeviceHealth::FAILED;
        return false;
    }

    if (info.lastRecoveryAttemptMs == 0)
    {
        return true;
    }

    return elapsedSince(info.lastRecoveryAttemptMs, nowMs) >= recoveryIntervalMs_;
}

void RecoverableDevice::markRecoveryAttempt(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);
    if (info.healthState == DeviceHealth::FAILED)
    {
        return;
    }

    info.healthState = DeviceHealth::DEGRADED;
    info.lastRecoveryAttemptMs = nowMs;
}

void RecoverableDevice::markRecoverySuccess(SystemContext &ctx, uint32_t nowMs) const
{
    markReadSuccess(ctx, nowMs);
}

void RecoverableDevice::markRecoveryFailure(SystemContext &ctx, uint32_t nowMs) const
{
    DeviceHealthInfo &info = health(ctx);

    if (info.healthState == DeviceHealth::FAILED)
    {
        return;
    }

    info.healthState = DeviceHealth::DEGRADED;
    info.lastRecoveryAttemptMs = nowMs;

    if (hasExceededRecoveryBudget(info, nowMs))
    {
        info.healthState = DeviceHealth::FAILED;
    }
}

bool RecoverableDevice::isAvailable(SystemContext &ctx) const
{
    const DeviceHealth state = health(ctx).healthState;
    return state == DeviceHealth::NORMAL || state == DeviceHealth::DEGRADED;
}

bool RecoverableDevice::isFailed(SystemContext &ctx) const
{
    return health(ctx).healthState == DeviceHealth::FAILED;
}

uint32_t RecoverableDevice::elapsedSince(uint32_t startMs, uint32_t nowMs)
{
    return nowMs - startMs;
}

bool RecoverableDevice::hasExceededRecoveryBudget(const DeviceHealthInfo &info, uint32_t nowMs) const
{
    if (info.lastOkMs == 0)
    {
        return false;
    }

    return elapsedSince(info.lastOkMs, nowMs) >= maxRetryBeforeFailureMs_;
}
