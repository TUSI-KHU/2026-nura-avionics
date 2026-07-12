#include "nura/core/executive.h"

namespace nura::core
{
namespace c = nura::contracts;

AppExecutionHealthSlot &Executive::healthSlot(c::AppId id)
{
    const size_t index = static_cast<size_t>(static_cast<uint16_t>(id));
    AppExecutionHealthSlot &slot = health_[index < health_.size() ? index : 0U];
    if (slot.id.load(std::memory_order_relaxed) == c::AppId::UNKNOWN)
    {
        slot.id.store(id, std::memory_order_relaxed);
    }
    return slot;
}

bool Executive::health(c::AppId id, AppExecutionHealth &snapshot) const
{
    const size_t index = static_cast<size_t>(static_cast<uint16_t>(id));
    if (index >= health_.size() ||
        health_[index].id.load(std::memory_order_acquire) != id)
    {
        return false;
    }
    const AppExecutionHealthSlot &slot = health_[index];
    snapshot.id = id;
    snapshot.run_count = slot.run_count.load(std::memory_order_relaxed);
    snapshot.failure_count = slot.failure_count.load(std::memory_order_relaxed);
    snapshot.deadline_miss_count =
        slot.deadline_miss_count.load(std::memory_order_relaxed);
    snapshot.last_duration_us = slot.last_duration_us.load(std::memory_order_relaxed);
    snapshot.max_duration_us = slot.max_duration_us.load(std::memory_order_relaxed);
    snapshot.last_completed_us = slot.last_completed_us.load(std::memory_order_acquire);
    snapshot.last_result = slot.last_result.load(std::memory_order_relaxed);
    return true;
}

AppRunResult Executive::run(IExecutableApp &app, uint32_t now_ms, uint32_t cycle_id,
                            uint32_t deadline_budget_us, c::FlightState state)
{
    const uint64_t started_us = clock_.nowUs();
    c::TraceRecord begin{};
    begin.cycle_id = cycle_id;
    begin.correlation_id = cycle_id;
    begin.timestamp_us = started_us;
    begin.app = app.id();
    begin.event = c::TraceEvent::TASK_BEGIN;
    begin.state = state;
    (void)trace_map_.tryRecord(begin);

    const AppRunResult result = app.run(now_ms, cycle_id);
    const uint64_t completed_us = clock_.nowUs();
    const uint64_t elapsed_us_64 = completed_us >= started_us ? completed_us - started_us : 0U;
    const uint32_t elapsed_us = elapsed_us_64 > UINT32_MAX
                                    ? UINT32_MAX
                                    : static_cast<uint32_t>(elapsed_us_64);

    AppExecutionHealthSlot &slot = healthSlot(app.id());
    slot.run_count.fetch_add(1U, std::memory_order_relaxed);
    slot.last_result.store(result, std::memory_order_relaxed);
    slot.last_duration_us.store(elapsed_us, std::memory_order_relaxed);
    slot.last_completed_us.store(static_cast<uint32_t>(completed_us),
                                 std::memory_order_release);
    uint32_t observed_max = slot.max_duration_us.load(std::memory_order_relaxed);
    while (elapsed_us > observed_max &&
           !slot.max_duration_us.compare_exchange_weak(
               observed_max, elapsed_us, std::memory_order_relaxed,
               std::memory_order_relaxed))
    {
    }
    if (result == AppRunResult::FAILED)
    {
        slot.failure_count.fetch_add(1U, std::memory_order_relaxed);
    }

    c::TraceRecord end{};
    end.cycle_id = cycle_id;
    end.correlation_id = cycle_id;
    end.timestamp_us = completed_us;
    end.duration_us = elapsed_us;
    end.app = app.id();
    end.event = c::TraceEvent::TASK_END;
    end.state = state;
    end.result = static_cast<int32_t>(result);
    (void)trace_map_.tryRecord(end);

    if (deadline_budget_us > 0U && elapsed_us > deadline_budget_us)
    {
        slot.deadline_miss_count.fetch_add(1U, std::memory_order_relaxed);
        c::TraceRecord miss = end;
        miss.event = c::TraceEvent::TASK_DEADLINE_MISS;
        miss.detail = deadline_budget_us;
        (void)trace_map_.tryRecord(miss);
    }

    return result;
}

AppRunResult Executive::runIfEnabled(IExecutableApp &app, uint32_t now_ms,
                                     uint32_t cycle_id,
                                     uint32_t deadline_budget_us,
                                     c::FlightState state,
                                     uint64_t enabled_mask)
{
    if ((enabled_mask & c::appBit(app.id())) != 0U)
    {
        return run(app, now_ms, cycle_id, deadline_budget_us, state);
    }

    c::TraceRecord skipped{};
    skipped.cycle_id = cycle_id;
    skipped.correlation_id = cycle_id;
    skipped.timestamp_us = clock_.nowUs();
    skipped.app = app.id();
    skipped.event = c::TraceEvent::TASK_SKIPPED;
    skipped.state = state;
    (void)trace_map_.tryRecord(skipped);
    return AppRunResult::NO_INPUT;
}

} // namespace nura::core
