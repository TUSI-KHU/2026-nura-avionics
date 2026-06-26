#pragma once
#include <stdint.h>

#include "core/tasks.h"

enum class TaskHealth : uint8_t
{
    // NORMAL: 정상, DEGRADED: 읽기 실패로 감시 중, FAILED: 자동 복구 포기
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
    // 태스크별 복구 정책을 생성 시 설정한다.
    RecoverableTask(TaskCriticality criticality,
                    uint8_t readFailureThreshold,
                    uint8_t maxRetryBeforeFailure,
                    uint32_t recoveryIntervalMs);

    virtual ~RecoverableTask() = default;

    // 실제 하드웨어 재초기화/복구 로직은 Task에서 구현한다.
    virtual bool recover(uint32_t nowMs) = 0;

    // watchdog가 읽을 health 스냅샷이다.
    TaskHealthInfo &health();
    const TaskHealthInfo &health() const;

    // 센서 태스크는 RW 결과만 기록하고, health 전이는 watchdog가 결정한다.
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
