#include "scheduler.h"

#if defined(NURA_BENCH_SCHEDULER_TRACE)
#include <Arduino.h>
#include <string.h>

namespace
{
#if !defined(NURA_BENCH_SCHEDULER_SLOW_MS)
constexpr uint32_t kSlowTaskMs = 20UL;
#else
constexpr uint32_t kSlowTaskMs = NURA_BENCH_SCHEDULER_SLOW_MS;
#endif

bool tracedTask(const char *name)
{
    return name != nullptr &&
           (strcmp(name, "telemetry") == 0 || strcmp(name, "fsm") == 0);
}

void traceTaskBoundary(const char *event, const char *name, uint32_t nowMs)
{
    if (!Serial || name == nullptr)
    {
        return;
    }

    Serial.print("TASK_");
    Serial.print(event);
    Serial.print(" ");
    Serial.print(name);
    Serial.print(" ms=");
    Serial.println(nowMs);
}

void traceSlowTask(const char *name, uint32_t startMs, uint32_t durationMs, bool ok)
{
    if (!Serial || name == nullptr)
    {
        return;
    }

    Serial.print("TASK_SLOW ");
    Serial.print(name);
    Serial.print(" start=");
    Serial.print(startMs);
    Serial.print(" dt=");
    Serial.print(durationMs);
    Serial.print(" ok=");
    Serial.println(ok ? "1" : "0");
}
} // namespace
#endif

Scheduler::Scheduler()
    : count_(0)
{
    // 등록되지 않은 테스크를 명확히 구분하기 위해 전체를 0으로 초기화한다.
    for (uint8_t i = 0; i < kMaxTasks; ++i)
    {
        entries_[i].task = nullptr;
        entries_[i].nextRunMs = 0;
        entries_[i].active = false;
    }
}

bool Scheduler::add(Task &task)
{
    if (count_ >= kMaxTasks)
    {
        return false;
    }

    entries_[count_].task = &task;
    entries_[count_].nextRunMs = 0;
    entries_[count_].active = true;
    ++count_;
    return true;
}

bool Scheduler::init(uint32_t nowMs)
{
    // 모든 태스크의 init을 호출하고 첫 실행 기준 시각을 맞춘다.
    for (uint8_t i = 0; i < count_; ++i)
    {
        Entry &e = entries_[i];
        if (!e.active || e.task == nullptr)
        {
            continue;
        }

        if (!e.task->init())
        {
            return false;
        }

        e.nextRunMs = nowMs;
    }

    return true;
}

void Scheduler::tick(uint32_t nowMs)
{
    // 주기가 도래한 태스크만 실행해 cooperative scheduler처럼 동작한다.
    for (uint8_t i = 0; i < count_; ++i)
    {
        Entry &e = entries_[i];
        if (!e.active || e.task == nullptr)
        {
            continue;
        }

        if ((int32_t)(nowMs - e.nextRunMs) < 0)
        {
            continue;
        }

        const uint32_t period = e.task->periodMs();
#if defined(NURA_BENCH_SCHEDULER_TRACE)
        const char *taskName = e.task->name();
        const bool traceBoundary = tracedTask(taskName);
        if (traceBoundary)
        {
            traceTaskBoundary("BEGIN", taskName, nowMs);
        }
        const uint32_t taskStartMs = millis();
#endif
        const bool tickOk = e.task->tick(nowMs);
#if defined(NURA_BENCH_SCHEDULER_TRACE)
        const uint32_t taskDurationMs = millis() - taskStartMs;
        if (traceBoundary)
        {
            traceTaskBoundary("END", taskName, millis());
        }
        if (taskDurationMs >= kSlowTaskMs)
        {
            traceSlowTask(taskName, taskStartMs, taskDurationMs, tickOk);
        }
#endif
        if (!tickOk)
        {
            // tick 실패 태스크는 반복 에러를 막기 위해 비활성화한다.
            e.active = false;
        }
        e.nextRunMs += period;

        if ((int32_t)(nowMs - e.nextRunMs) >= 0)
        {
            // 주기가 크게 밀렸다면 누적 지연 대신 현재 시각 기준으로 다시 동기화한다.
            e.nextRunMs = nowMs + period;
        }
    }
}
