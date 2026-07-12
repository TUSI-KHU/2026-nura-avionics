#pragma once

#include "nura/flight/acceleration_selector.h"
#include "nura/flight/state_app.h"

namespace nura::flight
{

class LaunchDetectorApp final : public IStateApp
{
public:
    nura::contracts::AppId id() const override;
    nura::contracts::FlightState state() const override;
    StateAppOutput onEnter(const nura::contracts::FlightStatus &status,
                           const nura::contracts::FlightInputs &inputs,
                           uint32_t now_ms) override;
    StateAppOutput step(const nura::contracts::FlightStatus &status,
                        const nura::contracts::FlightInputs &inputs,
                        uint32_t now_ms) override;

private:
    AccelerationSelector selector_{};
    uint8_t confirmation_count_ = 0U;
};

} // namespace nura::flight
