#include "core/recoverable_task/recoverable_task.h"

RecoverableTask::RecoverableTask(TaskCriticality criticality,
                                 uint8_t readFailureThreshold,
                                 uint8_t maxRetryBeforeFailure,
                                 uint32_t recoveryIntervalMs)
    : consecutiveReadFails_(0),
      hasSuccessfulReadSinceFailure_(false),
      criticality_(criticality),
      readFailureThreshold_(readFailureThreshold),
      maxRetryBeforeFailure_(maxRetryBeforeFailure),
      recoveryIntervalMs_(recoveryIntervalMs) {}

TaskHealthInfo &RecoverableTask::health()
{
    return health_;
}

const TaskHealthInfo &RecoverableTask::health() const
{
    return health_;
}

void RecoverableTask::markInitialized()
{
    health_.healthState = TaskHealth::NORMAL;
    health_.readFails = 0;
    health_.recoveryAttempts = 0;
    health_.lastRecoveryAttemptMs = 0;
    consecutiveReadFails_ = 0;
    hasSuccessfulReadSinceFailure_ = true;
}

void RecoverableTask::markReadSuccess()
{
    consecutiveReadFails_ = 0;
    hasSuccessfulReadSinceFailure_ = true;
}

void RecoverableTask::markReadFailure()
{
    if (consecutiveReadFails_ < 255)
    {
        ++consecutiveReadFails_;
    }
    hasSuccessfulReadSinceFailure_ = false;
}

bool RecoverableTask::shouldDegrade() const
{
    return health_.healthState == TaskHealth::NORMAL && consecutiveReadFails_ >= readFailureThreshold_;
}

void RecoverableTask::markDegraded()
{
    if (health_.healthState == TaskHealth::FAILED)
    {
        return;
    }

    health_.healthState = TaskHealth::DEGRADED;
    health_.readFails = consecutiveReadFails_;
}

bool RecoverableTask::hasRecoveredRead() const
{
    return health_.healthState == TaskHealth::DEGRADED && hasSuccessfulReadSinceFailure_;
}

bool RecoverableTask::shouldRecover(uint32_t nowMs) const
{
    if (health_.healthState != TaskHealth::DEGRADED)
    {
        return false;
    }

    if (consecutiveReadFails_ < readFailureThreshold_)
    {
        return false;
    }

    if (hasSuccessfulReadSinceFailure_)
    {
        return false;
    }

    if (health_.recoveryAttempts == 0)
    {
        return true;
    }

    return nowMs - health_.lastRecoveryAttemptMs >= recoveryIntervalMs_;
}

void RecoverableTask::markRecoveryAttempt(uint32_t nowMs)
{
    if (health_.healthState == TaskHealth::FAILED)
    {
        return;
    }

    health_.healthState = TaskHealth::DEGRADED;
    health_.readFails = consecutiveReadFails_;
    if (health_.recoveryAttempts < 255)
    {
        ++health_.recoveryAttempts;
    }
    health_.lastRecoveryAttemptMs = nowMs;
}

void RecoverableTask::markRecoverySuccess()
{
    markInitialized();
}

void RecoverableTask::markRecoveryFailure(uint32_t nowMs)
{
    if (health_.healthState == TaskHealth::FAILED)
    {
        return;
    }

    health_.healthState = TaskHealth::DEGRADED;
    health_.readFails = consecutiveReadFails_;
    health_.lastRecoveryAttemptMs = nowMs;

    if (hasExhaustedRecoveryAttempts())
    {
        health_.healthState = TaskHealth::FAILED;
    }
}

bool RecoverableTask::isAvailable() const
{
    const TaskHealth state = health_.healthState;
    return state == TaskHealth::NORMAL || state == TaskHealth::DEGRADED;
}

bool RecoverableTask::isFailed() const
{
    return health_.healthState == TaskHealth::FAILED;
}

TaskCriticality RecoverableTask::criticality() const
{
    return criticality_;
}

bool RecoverableTask::hasExhaustedRecoveryAttempts() const
{
    return health_.recoveryAttempts >= maxRetryBeforeFailure_;
}
