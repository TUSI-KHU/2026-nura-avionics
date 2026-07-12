#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "nura/contracts/common.h"
#include "nura/contracts/flight_topics.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/trace_map.h"

namespace nura::core
{

enum class AppRunResult : int32_t
{
    OK = 0,
    NO_INPUT = 1,
    DEGRADED = 2,
    FAILED = -1,
};

class IExecutableApp
{
public:
    virtual ~IExecutableApp() = default;
    virtual nura::contracts::AppId id() const = 0;
    virtual AppRunResult run(uint32_t now_ms, uint32_t cycle_id) = 0;
};

struct AppExecutionHealth
{
    nura::contracts::AppId id = nura::contracts::AppId::UNKNOWN;
    uint32_t run_count = 0U;
    uint32_t failure_count = 0U;
    uint32_t deadline_miss_count = 0U;
    uint32_t last_duration_us = 0U;
    uint32_t max_duration_us = 0U;
    uint64_t last_completed_us = 0U;
    AppRunResult last_result = AppRunResult::OK;
};

struct AppExecutionHealthSlot
{
    std::atomic<nura::contracts::AppId> id{nura::contracts::AppId::UNKNOWN};
    std::atomic<uint32_t> run_count{0U};
    std::atomic<uint32_t> failure_count{0U};
    std::atomic<uint32_t> deadline_miss_count{0U};
    std::atomic<uint32_t> last_duration_us{0U};
    std::atomic<uint32_t> max_duration_us{0U};
    std::atomic<uint32_t> last_completed_us{0U};
    std::atomic<AppRunResult> last_result{AppRunResult::OK};
};

class Executive
{
public:
    Executive(IMonotonicClock &clock, SystemTraceMap &trace_map)
        : clock_(clock), trace_map_(trace_map) {}

    AppRunResult run(IExecutableApp &app, uint32_t now_ms, uint32_t cycle_id,
                     uint32_t deadline_budget_us, nura::contracts::FlightState state);
    AppRunResult runIfEnabled(IExecutableApp &app, uint32_t now_ms,
                              uint32_t cycle_id, uint32_t deadline_budget_us,
                              nura::contracts::FlightState state,
                              uint64_t enabled_mask);
    bool health(nura::contracts::AppId id, AppExecutionHealth &snapshot) const;

private:
    AppExecutionHealthSlot &healthSlot(nura::contracts::AppId id);

    IMonotonicClock &clock_;
    SystemTraceMap &trace_map_;
    std::array<AppExecutionHealthSlot, 40U> health_{};
};

} // namespace nura::core
