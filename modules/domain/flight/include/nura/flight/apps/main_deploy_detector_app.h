#pragma once

#include "nura/flight/state_app.h"

namespace nura::flight
{

class MainDeployDetectorApp final : public IStateApp
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
    bool barometerUsable(const nura::contracts::BarometerSample &barometer,
                         uint32_t now_ms) const;
    void resetStuckMonitor();
    void trackStuck(uint32_t sample_time_ms, float altitude_m,
                    StateAppOutput &output);

    uint32_t last_barometer_time_ms_ = 0U;
    bool stuck_window_active_ = false;
    uint32_t stuck_window_start_ms_ = 0U;
    uint32_t last_stuck_sample_ms_ = 0U;
    float stuck_min_altitude_m_ = 0.0f;
    float stuck_max_altitude_m_ = 0.0f;
};

} // namespace nura::flight
