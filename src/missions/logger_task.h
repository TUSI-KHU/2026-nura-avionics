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
    ILogOutput &output_;
};
