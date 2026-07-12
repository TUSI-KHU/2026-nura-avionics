#pragma once

#include <array>
#include <cstdint>

#include "nura/flight/flight_policy.h"
#include "nura/flight/state_app.h"

namespace nura::flight
{

class ApogeeDetectorApp final : public IStateApp
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
    struct AltitudeSample
    {
        uint32_t sample_time_ms = 0U;
        float altitude_m = 0.0f;
    };

    struct QuadraticFit
    {
        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
        float rmse_m = 0.0f;
    };

    bool consumeBarometer(const nura::contracts::BarometerSample &barometer,
                          StateAppOutput &output);
    bool attitudeFallback(const nura::contracts::FlightStatus &status,
                          const nura::contracts::FlightInputs &inputs,
                          uint32_t now_ms, StateAppOutput &output);
    bool predictionReady(const nura::contracts::BarometerSample &barometer,
                         float current_altitude_m, StateAppOutput &output);
    bool pushPrediction(float prediction_m);
    bool plusTwoSigma(float &prediction_m) const;
    bool solveQuadratic(QuadraticFit &fit) const;
    static bool solve3x3(float matrix[3][4], float &x0, float &x1, float &x2);
    void pushAltitude(uint32_t sample_time_ms, float altitude_m);
    void resetPrediction();
    void resetStuckMonitor();
    void trackStuck(uint32_t sample_time_ms, float altitude_m, StateAppOutput &output);

    std::array<AltitudeSample, FlightPolicy::kApogeeFitWindowSamples> altitude_samples_{};
    std::array<float, FlightPolicy::kApogeePredictionHistorySamples> predictions_{};
    uint8_t altitude_head_ = 0U;
    uint8_t altitude_count_ = 0U;
    uint8_t prediction_head_ = 0U;
    uint8_t prediction_count_ = 0U;
    uint8_t prediction_confirmation_count_ = 0U;
    uint8_t descent_confirmation_count_ = 0U;
    uint8_t attitude_confirmation_count_ = 0U;
    uint32_t last_barometer_time_ms_ = 0U;
    uint32_t last_attitude_time_ms_ = 0U;
    float max_coast_altitude_m_ = 0.0f;

    bool stuck_window_active_ = false;
    uint32_t stuck_window_start_ms_ = 0U;
    uint32_t last_stuck_sample_ms_ = 0U;
    float stuck_min_altitude_m_ = 0.0f;
    float stuck_max_altitude_m_ = 0.0f;
};

} // namespace nura::flight
