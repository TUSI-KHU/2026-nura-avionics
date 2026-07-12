#pragma once

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"
#include "nura/platform/ports.h"

namespace nura::apps
{

class SupervisorApp final : public nura::core::IExecutableApp
{
public:
    SupervisorApp(nura::core::Executive &executive,
                  nura::core::ISupervisorBus &bus,
                  nura::core::IMonotonicClock &clock,
                  nura::platform::IWatchdog *watchdog = nullptr)
        : executive_(executive), bus_(bus), clock_(clock), watchdog_(watchdog) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;

private:
    bool appHealthy(nura::contracts::AppId app, uint32_t now_us,
                    uint32_t max_age_us) const;

    nura::core::Executive &executive_;
    nura::core::ISupervisorBus &bus_;
    nura::core::IMonotonicClock &clock_;
    nura::platform::IWatchdog *watchdog_;
    uint32_t sequence_ = 0U;
    bool watchdog_initialized_ = false;
};

} // namespace nura::apps
