#include "logger_task.h"

LoggerTask::LoggerTask(ILogOutput &output)
    : output_(output) {}

const char *LoggerTask::name() const
{
    return "logger";
}

bool LoggerTask::init(SystemContext &ctx)
{
    // prevent unused variables 
    (void)ctx;
    return true;
}

bool LoggerTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    // prevent unused variables 
    (void)nowMs;

    LogEntry entry;
    uint8_t drained = 0;

    while (drained < kDrainBudget && ctx.logger.pop(entry))
    {
        output_.write(entry);
        ++drained;
    }

    return true;
}

uint32_t LoggerTask::periodMs() const
{
    return 20;
}
