#pragma once

#include <array>
#include <cstdint>

#include "nura/flight/flight_policy.h"
#include "nura/flight/state_app.h"

namespace nura::flight
{

class LandingSequenceApp final : public IStateApp
{
public:
    explicit LandingSequenceApp(
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
    struct LandingSample
    {
        uint32_t sample_time_ms = 0U;
        float altitude_m = 0.0f;
    };

    bool consumeBarometer(const nura::contracts::BarometerSample &barometer);
    void pushSample(uint32_t sample_time_ms, float altitude_m);
    bool stable() const;
    void resetSamples();

    std::array<LandingSample, FlightPolicy::kLandingStableWindowSamples> samples_{};
    uint8_t head_ = 0U;
    uint8_t count_ = 0U;
    uint32_t last_barometer_time_ms_ = 0U;
    bool main_off_ = false;
};

} // namespace nura::flight
