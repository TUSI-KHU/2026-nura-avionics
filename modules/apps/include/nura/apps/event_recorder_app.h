#pragma once

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"
#include "nura/platform/ports.h"

namespace nura::apps
{

class EventRecorderApp final : public nura::core::IExecutableApp
{
public:
    EventRecorderApp(nura::core::IEventRecorderBus &bus,
                     nura::core::IMonotonicClock &clock,
                     nura::platform::IEventSink &sink)
        : bus_(bus), clock_(clock), sink_(sink) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;

private:
    nura::core::IEventRecorderBus &bus_;
    nura::core::IMonotonicClock &clock_;
    nura::platform::IEventSink &sink_;
};

} // namespace nura::apps
