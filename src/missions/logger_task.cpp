#include "logger_task.h"

LoggerTask::LoggerTask(ILogOutput &output)
    : output_(output) {}

const char *LoggerTask::name() const
{
    return "logger";
}

bool LoggerTask::init(SystemContext &ctx)
{
    lastQueueDroppedCount_ = ctx.logger.droppedCount();
    outputDroppedCount_ = 0;
    consecutiveOutputFails_ = 0;
    ctx.health.logOk = true;
    return true;
}

bool LoggerTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    const bool prevLogOk = ctx.health.logOk;
    bool queueOverflowed = false;

    const uint32_t queueDropped = ctx.logger.droppedCount();
    if (queueDropped > lastQueueDroppedCount_)
    {
        queueOverflowed = true;
        lastQueueDroppedCount_ = queueDropped;
    }

    LogEntry entry;
    uint8_t drained = 0;

    while (drained < kDrainBudget && ctx.logger.pop(entry))
    {
        if (!output_.write(entry))
        {
            ++outputDroppedCount_;
            if (consecutiveOutputFails_ < 255)
            {
                ++consecutiveOutputFails_;
            }
        }
        else
        {
            consecutiveOutputFails_ = 0;
        }
        ++drained;
    }

    const bool outputHealthy = consecutiveOutputFails_ < kOutputFailThreshold;
    ctx.health.logOk = outputHealthy && !queueOverflowed;

    return true;
}

uint32_t LoggerTask::periodMs() const
{
    return 20;
}
