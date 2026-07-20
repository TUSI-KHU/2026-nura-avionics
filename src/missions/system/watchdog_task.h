#pragma once

#include <stddef.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/tasks.h"
#include "state/abort_state.h"

class WatchdogTask : public Task
{
public:
    // 복구 가능한 태스크를 순회하며 health 전이와 abort 판단을 중앙에서 처리한다.
    WatchdogTask(RecoverableTask *const *devices, size_t deviceCount, AbortState &abortState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void handleRecovery(RecoverableTask &device, uint32_t nowMs) const;
    void handleAbort(RecoverableTask &device, uint32_t nowMs) const;

    RecoverableTask *const *devices_;
    size_t deviceCount_;
    AbortState &abortState_;
    Logger &logger_;
    const IAppConfig &config_;
};
