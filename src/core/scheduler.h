#pragma once

#include <stdint.h>
#include "contexts.h"
#include "tasks.h"

class Scheduler
{
public:
    Scheduler();

    bool add(Task &task);
    bool init(SystemContext &ctx, uint32_t nowMs);
    void tick(SystemContext &ctx, uint32_t nowMs);

private:
    static const uint8_t kMaxTasks = 10;

    struct Entry
    {
        Task *task;
        uint32_t nextRunMs;
        bool active;
    };

    Entry entries_[kMaxTasks];
    uint8_t count_;
};
