#pragma once

#include "nura/flight/flight_policy.h"
#include "nura/flight/state_app.h"

namespace nura::flight
{

class DrogueSequenceApp final : public IStateApp
{
public:
    explicit DrogueSequenceApp(
        const RecoveryTimingPolicy &timing = kRecoveryTimingPolicy)
        : timing_(timing) {}
    nura::contracts::AppId id() const override;
    nura::contracts::FlightState state() const override;
    StateAppOutput onEnter(const nura::contracts::FlightStatus &status,
                           const nura::contracts::FlightInputs &inputs,
                           uint32_t now_ms) override;
    StateAppOutput step(const nura::contracts::FlightStatus &status,
                        const nura::contracts::FlightInputs &inputs,
                        uint32_t now_ms) override;

private:
    const RecoveryTimingPolicy &timing_;
    bool primary_off_ = false;
    bool backup_on_ = false;
    bool backup_off_ = false;
};

} // namespace nura::flight
