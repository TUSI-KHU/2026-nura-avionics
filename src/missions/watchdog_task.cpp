#include "watchdog_task.h"

WatchdogTask::WatchdogTask(RecoverableTask *const *devices, size_t deviceCount)
    : devices_(devices),
      deviceCount_(deviceCount) {}

const char *WatchdogTask::name() const
{
    return "watchdog";
}

bool WatchdogTask::init(SystemContext &ctx)
{
    ctx.abort.active = false;
    ctx.abort.reason = AbortReason::NONE;
    ctx.abort.raisedMs = 0;
    return true;
}

bool WatchdogTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    for (size_t i = 0; i < deviceCount_; ++i)
    {
        RecoverableTask *const device = devices_[i];
        if (device == 0)
        {
            continue;
        }

        handleRecovery(ctx, *device, nowMs);
        handleAbort(ctx, *device, nowMs);
    }

    return true;
}

uint32_t WatchdogTask::periodMs() const
{
    return 50U;
}

void WatchdogTask::handleRecovery(SystemContext &ctx, RecoverableTask &device, uint32_t nowMs) const
{
    if (device.shouldDegrade())
    {
        device.markDegraded();
    }

    if (device.hasRecoveredRead())
    {
        device.markRecoverySuccess();
        LOGI(ctx.logger, nowMs, "watchdog", "recovery succeeded");
        return;
    }

    if (!device.shouldRecover(nowMs))
    {
        return;
    }

    LOGW(ctx.logger, nowMs, "watchdog", "attempting recovery");
    device.markRecoveryAttempt(nowMs);

    if (device.recover(nowMs))
    {
        device.markRecoverySuccess();
        LOGI(ctx.logger, nowMs, "watchdog", "recovery succeeded");
        return;
    }

    device.markRecoveryFailure(nowMs);
    LOGE(ctx.logger, nowMs, "watchdog", "recovery failed");
}

void WatchdogTask::handleAbort(SystemContext &ctx, RecoverableTask &device, uint32_t nowMs) const
{
    if (ctx.abort.active || !device.isFailed() || device.criticality() != TaskCriticality::CRITICAL)
    {
        return;
    }

    ctx.abort.active = true;
    ctx.abort.reason = AbortReason::CRITICAL_SENSOR_FAILURE;
    ctx.abort.raisedMs = nowMs;
}
