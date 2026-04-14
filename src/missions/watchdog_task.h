#pragma once

#include <stddef.h>

#include "core/contexts.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/tasks.h"

class WatchdogTask : public Task
{
public:
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
