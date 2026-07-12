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
    // 초기화 또는 복구 완료 시 health와 관측값를 모두 초기 상태로 되돌린다.
    health_.healthState = TaskHealth::NORMAL;
    health_.readFails = 0;
    health_.recoveryAttempts = 0;
    health_.lastRecoveryAttemptMs = 0;
    consecutiveReadFails_ = 0;
    hasSuccessfulReadSinceFailure_ = true;
}

void RecoverableTask::markReadSuccess()
{
    // 정상 읽기는 관측 누적만 초기화하고, health 복구 확정은 watchdog가 수행한다.
    consecutiveReadFails_ = 0;
    hasSuccessfulReadSinceFailure_ = true;
}

void RecoverableTask::markReadFailure()
{
    // 읽기 실패 관측만 누적하고, DEGRADED 전이는 watchdog가 임계치 기준으로 수행한다.
    if (consecutiveReadFails_ < 255)
    {
        ++consecutiveReadFails_;
    }
    hasSuccessfulReadSinceFailure_ = false;
}

bool RecoverableTask::shouldDegrade() const
{
    // NORMAL 상태에서 실패 임계치를 처음 넘긴 순간만 DEGRADED 전이를 허용한다.
    return health_.healthState == TaskHealth::NORMAL && consecutiveReadFails_ >= readFailureThreshold_;
}

void RecoverableTask::markDegraded()
{
    // 현재 누적 실패 횟수를 health 정보에 저장한다.
    if (health_.healthState == TaskHealth::FAILED)
    {
        return;
    }

    health_.healthState = TaskHealth::DEGRADED;
    health_.readFails = consecutiveReadFails_;
}

bool RecoverableTask::hasRecoveredRead() const
{
    // DEGRADED 상태에서 복구가 성공했는지 watchdog가 확인한다.
    return health_.healthState == TaskHealth::DEGRADED && hasSuccessfulReadSinceFailure_;
}

bool RecoverableTask::shouldRecover(uint32_t nowMs) const
{
    // 충분한 읽기 실패가 누적됐고 읽기가 실패했을 때만 복구를 시도한다.
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
        // 첫 복구 시도는 interval을 기다리지 않고 바로 허용한다.
        return true;
    }

    return nowMs - health_.lastRecoveryAttemptMs >= recoveryIntervalMs_;
}

void RecoverableTask::markRecoveryAttempt(uint32_t nowMs)
{
    // 복구 시도 횟수와 시각을 기록해 다음 복구 가능 시점을 계산한다.
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
    // 복구 실패 후 재시도 횟수를 초과하면 FAILED로 처리한다.
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
    // 허용된 자동 복구 시도 횟수를 모두 사용했는지 확인한다.
    return health_.recoveryAttempts >= maxRetryBeforeFailure_;
}
