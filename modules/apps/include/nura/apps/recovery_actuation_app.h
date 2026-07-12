#pragma once

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"
#include "nura/platform/ports.h"

namespace nura::apps
{

class RecoveryActuationApp final : public nura::core::IExecutableApp
{
public:
    RecoveryActuationApp(nura::platform::IRecoveryOutput &output,
                         nura::core::IRecoveryBus &bus,
                         nura::core::IMonotonicClock &clock,
                         nura::core::SystemTraceMap &trace_map)
        : output_(output), bus_(bus), clock_(clock), trace_map_(trace_map) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;

private:
    bool authorized(const nura::contracts::ActuationIntent &intent) const;
    void traceResult(const nura::contracts::ActuationIntent &intent,
                     uint32_t cycle_id, uint64_t now_us, int32_t result);

    nura::platform::IRecoveryOutput &output_;
    nura::core::IRecoveryBus &bus_;
    nura::core::IMonotonicClock &clock_;
    nura::core::SystemTraceMap &trace_map_;
    nura::contracts::FlightStatus status_{};
    bool have_status_ = false;
    bool initialized_ = false;
};

} // namespace nura::apps
