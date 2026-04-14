#pragma once
#include <stdint.h>

#include "core/tasks.h"

enum class TaskHealth : uint8_t
{
    NORMAL,
    DEGRADED,
    FAILED
};

enum class TaskCriticality : uint8_t
{
    NON_CRITICAL,
    CRITICAL
};

struct TaskHealthInfo
{
    TaskHealth healthState = TaskHealth::FAILED;
    uint8_t readFails = 0;
    uint8_t recoveryAttempts = 0;
    uint32_t lastRecoveryAttemptMs = 0;
};

class RecoverableTask : public Task
{
public:
    RecoverableTask(TaskCriticality criticality,
                    uint8_t readFailureThreshold,
                    uint8_t maxRetryBeforeFailure,
                    uint32_t recoveryIntervalMs);

    virtual ~RecoverableTask() = default;

    virtual bool recover(uint32_t nowMs) = 0;

    TaskHealthInfo &health();
    const TaskHealthInfo &health() const;

    void markInitialized();
    void markReadSuccess();
    void markReadFailure();
    bool shouldDegrade() const;
    void markDegraded();
    bool hasRecoveredRead() const;

    bool shouldRecover(uint32_t nowMs) const;
    void markRecoveryAttempt(uint32_t nowMs);
    void markRecoverySuccess();
    void markRecoveryFailure(uint32_t nowMs);

    bool isAvailable() const;
    bool isFailed() const;
    TaskCriticality criticality() const;

private:
    bool hasExhaustedRecoveryAttempts() const;
    TaskHealthInfo health_;
    uint8_t consecutiveReadFails_;
    bool hasSuccessfulReadSinceFailure_;

    TaskCriticality criticality_;
    uint8_t readFailureThreshold_;
    uint8_t maxRetryBeforeFailure_;
    uint32_t recoveryIntervalMs_;
};
