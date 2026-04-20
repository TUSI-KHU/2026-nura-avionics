#pragma once

#include <stddef.h>

#include "core/contexts.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/tasks.h"

class WatchdogTask : public Task
{
public:
    // 복구 가능한 태스크를 순회하며 health 전이와 abort 판단을 중앙에서 처리한다.
    WatchdogTask(RecoverableTask *const *devices, size_t deviceCount);

    const char *name() const override;
    bool init(SystemContext &ctx) override;
    bool tick(SystemContext &ctx, uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void handleRecovery(SystemContext &ctx, RecoverableTask &device, uint32_t nowMs) const;
    void handleAbort(SystemContext &ctx, RecoverableTask &device, uint32_t nowMs) const;

    RecoverableTask *const *devices_;
    size_t deviceCount_;
};
