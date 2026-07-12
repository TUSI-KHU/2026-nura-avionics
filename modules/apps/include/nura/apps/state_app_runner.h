#pragma once

#include "nura/core/monotonic_clock.h"
#include "nura/core/trace_map.h"
#include "nura/flight/state_app.h"

namespace nura::apps
{

class TracingStateAppRunner final : public nura::flight::IStateAppRunner
{
public:
    TracingStateAppRunner(nura::core::IMonotonicClock &clock,
                          nura::core::SystemTraceMap &trace_map)
        : clock_(clock), trace_map_(trace_map) {}

    nura::flight::StateAppOutput enter(
        nura::flight::IStateApp &app,
        const nura::contracts::FlightStatus &status,
        const nura::contracts::FlightInputs &inputs,
        uint32_t now_ms, uint32_t cycle_id) override;
    nura::flight::StateAppOutput step(
        nura::flight::IStateApp &app,
        const nura::contracts::FlightStatus &status,
        const nura::contracts::FlightInputs &inputs,
        uint32_t now_ms, uint32_t cycle_id) override;

private:
    nura::flight::StateAppOutput run(
        nura::flight::IStateApp &app,
        const nura::contracts::FlightStatus &status,
        const nura::contracts::FlightInputs &inputs,
        uint32_t now_ms, uint32_t cycle_id, bool entering);
    void record(nura::contracts::TraceEvent event,
                nura::contracts::AppId app,
                nura::contracts::FlightState state,
                uint32_t cycle_id, uint64_t timestamp_us,
                uint32_t duration_us, uint32_t detail);

    nura::core::IMonotonicClock &clock_;
    nura::core::SystemTraceMap &trace_map_;
};

} // namespace nura::apps
