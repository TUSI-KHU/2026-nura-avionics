#pragma once

#include "core/contexts.h"
#include "core/tasks.h"
#include "core/logger/log_output.h"

class LoggerTask : public Task
{
public:
    explicit LoggerTask(ILogOutput &output);

    const char *name() const override;
    bool init(SystemContext &ctx) override;
    bool tick(SystemContext &ctx, uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    const uint8_t kDrainBudget = 4;
    const uint8_t kOutputFailThreshold = 3;

    uint32_t lastQueueDroppedCount_ = 0;
    uint32_t outputDroppedCount_ = 0;
    uint8_t consecutiveOutputFails_ = 0;

    ILogOutput &output_;
};
