#pragma once

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"

namespace nura::apps
{

class InputAggregatorApp final : public nura::core::IExecutableApp
{
public:
    InputAggregatorApp(nura::core::IInputAggregationBus &bus,
                       nura::core::IMonotonicClock &clock)
        : bus_(bus), clock_(clock) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;

private:
    nura::core::IInputAggregationBus &bus_;
    nura::core::IMonotonicClock &clock_;
    nura::contracts::LowGImuSample low_g_{};
    nura::contracts::HighGImuSample high_g_{};
    nura::contracts::BarometerSample barometer_{};
    nura::contracts::SensorHealthSnapshot health_{};
    nura::contracts::SafetyStatus safety_{};
    uint32_t sequence_ = 0U;
    bool have_input_ = false;
};

} // namespace nura::apps
