#pragma once

#include <stdint.h>
#include "contexts.h"
#include "tasks.h"

class Scheduler
{
public:
    Scheduler();

    // 미리 확보된 고정 크기 버퍼에 태스크를 등록한다.
    bool add(Task &task);
    bool init(SystemContext &ctx, uint32_t nowMs);
    void tick(SystemContext &ctx, uint32_t nowMs);

private:
    static const uint8_t kMaxTasks = 10;

    struct Entry
    {
        // 다음 실행 시각과 활성 여부를 각 태스크별로 추적한다.
        Task *task;
        uint32_t nextRunMs;
        bool active;
    };

    Entry entries_[kMaxTasks];
    uint8_t count_;
};
