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
    // 각 recoverable task를 독립적으로 검사해 복구와 중단 여부를 평가한다.
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
    // 읽기 실패가 임계치를 넘으면 watchdog가 health를 DEGRADED로 변경한다.
    if (device.shouldDegrade())
    {
        device.markDegraded();
    }

    // 정상 읽기가 다시 들어오면 watchdog가 recovery success를 확정하고 로그를 남긴다.
    if (device.hasRecoveredRead())
    {
        device.markRecoverySuccess();
        LOGI(ctx.logger, nowMs, "watchdog", "recovery succeeded");
        return;
    }

    // 능동 복구는 interval과 retry 예산 조건을 만족할 때만 시도한다.
    if (!device.shouldRecover(nowMs))
    {
        return;
    }

    LOGW(ctx.logger, nowMs, "watchdog", "attempting recovery");
    device.markRecoveryAttempt(nowMs);

    // recover 성공 시 health를 NORMAL로 되돌리고 성공 로그를 남긴다.
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
    // 현재 정책은 critical 태스크가 FAILED일 때만 전역 abort를 올린다.
    if (ctx.abort.active || !device.isFailed() || device.criticality() != TaskCriticality::CRITICAL)
    {
        return;
    }

    ctx.abort.active = true;
    ctx.abort.reason = AbortReason::CRITICAL_SENSOR_FAILURE;
    ctx.abort.raisedMs = nowMs;
}
