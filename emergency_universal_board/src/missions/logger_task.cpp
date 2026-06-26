#include "logger_task.h"

LoggerTask::LoggerTask(Logger &logger, ILogOutput &output, const IAppConfig &config)
    : logger_(logger),
      output_(output),
      config_(config) {}

const char *LoggerTask::name() const
{
    return "logger";
}

bool LoggerTask::init()
{
    // 현재 드롭 카운트를 기준점으로 잡아 이후 overflow를 감지한다.
    lastQueueDroppedCount_ = logger_.droppedCount();
    outputDroppedCount_ = 0;
    consecutiveOutputFails_ = 0;
    return true;
}

bool LoggerTask::tick(uint32_t nowMs)
{
    (void)nowMs;

    // 한 tick당 제한된 개수만 배출해 로그 출력이 다른 태스크를 막지 않게 한다.
    bool queueOverflowed = false;

    const uint32_t queueDropped = logger_.droppedCount();
    if (queueDropped > lastQueueDroppedCount_)
    {
        queueOverflowed = true;
        lastQueueDroppedCount_ = queueDropped;
    }

    const LogFlushResult flush = logger_.flushTo(output_, config_.loggerDrainBudget());
    outputDroppedCount_ += flush.outputFailures;

    if (flush.outputFailures > 0)
    {
        const uint8_t nextFailCount = consecutiveOutputFails_ + flush.outputFailures;
        consecutiveOutputFails_ = nextFailCount < consecutiveOutputFails_ ? 255U : nextFailCount;
    }
    else if (flush.drained > 0)
    {
        consecutiveOutputFails_ = 0;
    }

    const bool outputHealthy = consecutiveOutputFails_ < config_.loggerOutputFailThreshold();
    // 아직 health 컨텍스트에 연결하지 않았지만 진단용 값은 유지한다.
    (void)outputHealthy;
    (void)queueOverflowed;

    return true;
}

uint32_t LoggerTask::periodMs() const
{
    return config_.loggerTaskPeriodMs();
}
