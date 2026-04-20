#include "logger_task.h"

LoggerTask::LoggerTask(ILogOutput &output)
    : output_(output) {}

const char *LoggerTask::name() const
{
    return "logger";
}

bool LoggerTask::init(SystemContext &ctx)
{
    // 현재 드롭 카운트를 기준점으로 잡아 이후 overflow를 감지한다.
    lastQueueDroppedCount_ = ctx.logger.droppedCount();
    outputDroppedCount_ = 0;
    consecutiveOutputFails_ = 0;
    return true;
}

bool LoggerTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    // 한 tick당 제한된 개수만 배출해 로그 출력이 다른 태스크를 막지 않게 한다.
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
        // 출력 실패 시 드롭으로 처리한다.
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
    // 아직 health 컨텍스트에 연결하지 않았지만 진단용 값은 유지한다.
    (void)outputHealthy;
    (void)queueOverflowed;

    return true;
}

uint32_t LoggerTask::periodMs() const
{
    return 20U;
}
