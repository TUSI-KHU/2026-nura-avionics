// #include "watchdog_task.h"

// WatchdogTask::WatchdogTask(RecoverableDevice *const *devices, size_t deviceCount)
//     : devices_(devices),
//       deviceCount_(deviceCount) {}

// const char *WatchdogTask::name() const
// {
//     return "watchdog";
// }

// bool WatchdogTask::init(SystemContext &ctx)
// {
//     ctx.abort.active = false;
//     ctx.abort.reason = AbortReason::NONE;
//     ctx.abort.raisedMs = 0;
//     syncHealthFlags(ctx);
//     return true;
// }

// bool WatchdogTask::tick(SystemContext &ctx, uint32_t nowMs)
// {
//     for (size_t i = 0; i < deviceCount_; ++i)
//     {
//         RecoverableDevice *const device = devices_[i];
//         if (device == 0)
//         {
//             continue;
//         }

//         handleRecovery(ctx, *device, nowMs);
//         handleAbort(ctx, *device, nowMs);
//     }

//     return true;
// }

// uint32_t WatchdogTask::periodMs() const
// {
//     return 50U;
// }

// void WatchdogTask::handleRecovery(SystemContext &ctx, RecoverableDevice &device, uint32_t nowMs) const
// {
//     if (!device.shouldRecover(ctx, nowMs))
//     {
//         return;
//     }

//     LOGW(ctx.logger, nowMs, "watchdog", "attempting recovery");
//     device.markRecoveryAttempt(ctx, nowMs);

//     if (device.recover(ctx, nowMs))
//     {
//         device.markRecoverySuccess(ctx, nowMs);
//         LOGI(ctx.logger, nowMs, "watchdog", "recovery succeeded");
//         return;
//     }

//     device.markRecoveryFailure(ctx, nowMs);
//     LOGE(ctx.logger, nowMs, "watchdog", "recovery failed");
// }

// void WatchdogTask::handleAbort(SystemContext &ctx, RecoverableDevice &device, uint32_t nowMs) const
// {
//     if (ctx.abort.active || !device.isFailed(ctx))
//     {
//         return;
//     }

//     // TODO: Implement abort
// }
