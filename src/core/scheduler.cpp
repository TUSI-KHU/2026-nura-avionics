#include "scheduler.h"

Scheduler::Scheduler()
    : count_(0)
{
    for (uint8_t i = 0; i < kMaxTasks; ++i)
    {
        entries_[i].task = 0;
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

bool Scheduler::init(SystemContext &ctx, uint32_t nowMs)
{
    ctx.health.schedulerOk = true;

    for (uint8_t i = 0; i < count_; ++i)
    {
        Entry &e = entries_[i];
        if (!e.active || e.task == 0)
        {
            continue;
        }

        if (!e.task->init(ctx))
        {
            ctx.health.schedulerOk = false;
            return false;
        }

        e.nextRunMs = nowMs;
    }

    return true;
}

void Scheduler::tick(SystemContext &ctx, uint32_t nowMs)
{
    for (uint8_t i = 0; i < count_; ++i)
    {
        Entry &e = entries_[i];
        if (!e.active || e.task == 0)
        {
            continue;
        }

        if ((int32_t)(nowMs - e.nextRunMs) < 0)
        {
            continue;
        }

        const uint32_t period = e.task->periodMs();
        if (!e.task->tick(ctx, nowMs))
        {
            ctx.health.schedulerOk = false;
        }
        e.nextRunMs += period;

        if ((int32_t)(nowMs - e.nextRunMs) >= 0)
        {
            e.nextRunMs = nowMs + period;
        }
    }
}
