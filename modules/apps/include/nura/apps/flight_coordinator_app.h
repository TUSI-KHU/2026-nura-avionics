#pragma once

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"
#include "nura/flight/flight_coordinator.h"

namespace nura::apps
{

class FlightCoordinatorApp final : public nura::core::IExecutableApp
{
public:
    FlightCoordinatorApp(nura::flight::FlightCoordinator &coordinator,
                         nura::core::IFlightCoordinatorBus &bus,
                         nura::core::IMonotonicClock &clock,
                         nura::core::SystemTraceMap &trace_map)
        : coordinator_(coordinator), bus_(bus), clock_(clock), trace_map_(trace_map) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;

private:
    nura::core::AppRunResult publish(
        const nura::flight::CoordinatorOutput &output, uint32_t cycle_id,
        uint64_t now_us);
    nura::flight::FlightCoordinator &coordinator_;
    nura::core::IFlightCoordinatorBus &bus_;
    nura::core::IMonotonicClock &clock_;
    nura::core::SystemTraceMap &trace_map_;
    bool initialized_ = false;
};

} // namespace nura::apps
